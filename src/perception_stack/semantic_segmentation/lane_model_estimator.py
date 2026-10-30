#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import struct
import colorsys
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from custom_interfaces.msg import LaneModel, DetectionArray, DetectionMeterArray
from sklearn.cluster import DBSCAN
from std_msgs.msg import Bool, Float32

class QuadraticLaneEstimator(Node):
    def __init__(self):
        super().__init__('quadratic_lane_estimator')
        
        # Parameters
        self.expected_width = 2.8     
        self.min_points = 5           
        self.cluster_tol = 0.8         
        self.max_yaw_angle = 0.69       # Initial orientation limit

        # Temporal filter
        self.d_lat_last = 10000
        self.max_d_lat_change = 0.5 #s Maximum allowed change in d_lat between frames (in meters)
        self.detection = False
        self.alpha = True
        self.estructure = 0

        # Lane change:
        self.d_lat_left = None
        self.d_lat_right = None
        self.change_left = False # Indicator that a lane change has been requested 
        self.change_right = False # Indicator that a lane change has been requested 
        self.change_last_right = False # Last change right status 
        self.change_last_left = False # Current status of left lane change 
        self.change_n = 0 # Frame counter to confirm lane change
        self.detections_meter_list = []

        # Subscribers
        self.sub = self.create_subscription(PointCloud2, '/lane/meter_candidates', self.cb_points, 10)
        self.sub_detections = self.create_subscription(DetectionMeterArray, '/detection/meter_results', self.cb_detections, 10)
        self.sub_alpha = self.create_subscription(Float32, '/alpha/vision', self.cb_alpha, 10)
        self.create_subscription(Bool, '/manual', self.manual_mode_cb, 10)
        self.create_subscription(Bool, '/change/left', self.change_lane_left_cb, 10)
        self.create_subscription(Bool, '/change/right', self.change_lane_right_cb, 10)

        # Publishers
        self.pub_model = self.create_publisher(LaneModel, '/lane/model_raw', 10)
        self.pub_model_left = self.create_publisher(LaneModel, '/lane/model_left', 10)
        self.pub_model_right = self.create_publisher(LaneModel, '/lane/model_right', 10)

    def create_pc2_msg(self, header, points_rgb):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points_rgb)
    
    def cb_detections(self, msg):
        self.detections_meter_list = msg.detections
        tiene_6 = False
        tiene_7 = False
        
        for det in msg.detections:
            if det.class_id == 6:
                tiene_6 = True
            elif det.class_id == 7:
                tiene_7 = True
        
        # detection is True only if there is a 7 but there is NO 6.
        self.detection = tiene_7 and not tiene_6

    def manual_mode_cb(self, msg):
        if msg.data:
            self.d_lat_last = 10000  # Reset temporary filter in manual mode
            
    def cb_alpha(self, msg):
        if msg.data <= 0.5:
            self.alpha = True
        else:
            self.alpha = False

    def change_lane_left_cb(self, msg):
        self.change_left = msg.data
        if not self.change_last_left and self.change_left:
            self.change_n = 0  # Reset counter when detecting start of lane change
        self.change_last_left = self.change_left

    def change_lane_right_cb(self, msg):
        self.change_right = msg.data
        if not self.change_last_right and self.change_right:
            self.change_n = 0  # Reset counter when detecting start of lane change
        self.change_last_right = self.change_right


    def cb_points(self, msg):
        pts = np.array([(p[0], p[1]) for p in point_cloud2.read_points(msg, field_names=['x', 'y'], skip_nans=True)])
        if len(pts) < self.min_points: return

        # We create a copy for clustering.
        pts_scaled = pts.copy()

        # We compress the x-axis using a root.
        pts_scaled[:, 0] = np.sqrt(pts[:, 0]) 

        # eps: Maximum distance between two samples for them to be considered neighbors
        # min_samples: The number of points required to form a "cluster"
        clustering = DBSCAN(eps=0.3, min_samples=5).fit(pts_scaled)
        labels = clustering.labels_
        
        unique_labels = set(labels)

        candidates = []
        mean = []
        
        for label in unique_labels:
            if label == -1: continue  # Skip the noise detected by DBSCAN
            
            # Mask to obtain only the points of this cluster
            cluster_mask = (labels == label)
            cluster_pts = pts[cluster_mask]
            
            if len(cluster_pts) < self.min_points: 
                continue

            # --- PERCENTAGE-BASED CLASSIFICATION LOGIC ---
            # We count how many points are to the left (y > 0) and how many to the right (y < 0)
            puntos_izq = np.sum(cluster_pts[:, 1] > 0)
            puntos_der = np.sum(cluster_pts[:, 1] < 0)
            total = len(cluster_pts)
            
            porcentaje_izq = (puntos_izq / total) * 100
            # We decide the side by simple majority (more than 50%)
            lado_predominante = "izq" if porcentaje_izq > 50 else "der"
            # ----------------------------------------------------
            
            # POLYNOMIAL FIT
            # We use x as the independent variable and 'y' as the dependent variable
            try:
                if max(cluster_pts[:, 0]) > 7.0 and len(cluster_pts) > 50:
                    coeffs = np.polyfit(cluster_pts[:, 0], cluster_pts[:, 1], 2)
                    c2, c1, c0 = coeffs
                else:
                    # For a small number of points, we fit a straight line (degree 1).
                    coeffs = np.polyfit(cluster_pts[:, 0], cluster_pts[:, 1], 1)
                    c1, c0 = coeffs
                    c2 = 0.0
                
                c0_corr = c0 * np.cos(np.arctan(c1))
                minimo = min(cluster_pts[:, 0])
                maximo = max(cluster_pts[:, 0])
                if abs(np.arctan(c1)) < self.max_yaw_angle and len(cluster_pts) > 10 and abs(maximo - minimo) > 1.0:
                    candidates.append({'cluster_id': label, 'lado': lado_predominante,'mean': np.mean(cluster_pts[:, 1]), 'c0': c0,'c0_corr': c0_corr,  'c1': c1, 'c2': c2, 'pts': cluster_pts, 'min_x': minimo})
            except np.RankWarning:
                continue

        if not candidates: return

        # 3. Selection of the best candidates using the 'side' label
        l_cands = [c for c in candidates if c['lado'] == "izq"]
        r_cands = [c for c in candidates if c['lado'] == "der"]

        best_l = min(l_cands, key=lambda x: abs(x['mean'])) if l_cands else None
        best_r = min(r_cands, key=lambda x: abs(x['mean'])) if r_cands else None

        aux = min(candidates, key=lambda x: abs(x['mean'])) if candidates else None

        if best_l and best_r and abs(best_l['c0'] - best_r['c0']) > (self.expected_width + 0.5):
            if abs(best_l['c0']) < abs(best_r['c0']):
                best_r = None
            else:                
                best_l = None
        
        #========================================================================================================================================
        if len(candidates) > 1:
            import itertools
            
            # Sort clusters from left to right (highest to lowest c0_corr)
            sorted_c = sorted(candidates, key=lambda x: x['c0_corr'], reverse=True)
            num_c = len(sorted_c)
            
            # Ideal ranges
            IDEAL_28 = 2.8
            IDEAL_42 = 4.2
            
            # STEP 1: Sort clusters
            #cluster_order = [f"{c['cluster_id']}({c['lado']}:{c['c0_corr']:.2f})" for c in sorted_c]
            
            # STEP 2: Determine widths between adjacent clusters
            lane_segments = []
            for i in range(num_c - 1):
                c_left = sorted_c[i]
                c_right = sorted_c[i+1]
                width = abs(c_left['c0_corr'] - c_right['c0_corr'])
                
                # Determine lane type
                if 2.3 <= width <= 3.5:
                    lane_type = "ESTANDAR_2.8"
                    belongs_to = "95%"
                elif 3.5 < width <= 5.2 and len(r_cands) == 1:
                    lane_type = "ANCHO_4.2"
                    belongs_to = "5%"
                elif 4.0 < width <= 5.2:
                    lane_type = "ANCHO_4.2"
                    belongs_to = "5%"
                else:
                    lane_type = "INVALIDO"
                    belongs_to = None
                
                segment_info = {
                    'left_idx': i,
                    'right_idx': i+1,
                    'left_cluster': c_left,
                    'right_cluster': c_right,
                    'width': width,
                    'type': lane_type,
                    'belongs_to': belongs_to
                }
                lane_segments.append(segment_info)
                
            # Auxiliary function to evaluate the consistency of a scene
            def evaluate_scene(segments, scene_name=""):
                valid_segments = [s for s in segments if s.get('type') != "INVALIDO" and s.get('width', 0) >= 2.3]
                
                if not valid_segments:
                    return None, "SIN_CARRILES_VALIDOS"
                
                # Get types of valid segments
                types = [s['belongs_to'] for s in valid_segments if s['belongs_to']]
                
                if not types:
                    return None, "SIN_TIPOS_VALIDOS"
                
                unique_types = set(types)
                
                if len(unique_types) == 1:
                    # All valid items of the same type
                    return unique_types.pop(), "CONSISTENTE"
                else:
                    # Mixed types
                    return None, "INCONSISTENTE"
            
            # CASE 1: 4 clusters
            if num_c == 4:                
                # Count valid segments by type
                valid_segments = [seg for seg in lane_segments if seg['type'] != "INVALIDO"]
                
                if len(valid_segments) >= 2:
                    # Count majority
                    count_95 = sum(1 for seg in valid_segments if seg['belongs_to'] == "95%")
                    count_5 = sum(1 for seg in valid_segments if seg['belongs_to'] == "5%")
                    
                    if count_95 > count_5:
                        # Majority 95%
                        scene_type = "95%"
                        self.estructure = 95
                    elif count_5 > count_95:
                        # Majority 5%
                        scene_type = "5%"
                        self.estructure = 5
                    else:
                        # Tie, search for alternative combinations                        
                        invalid_indices = [i for i, seg in enumerate(lane_segments) if seg['type'] == "INVALIDO"]
                        best_scene = None
                        best_type = None
                        best_score = float('inf')
                        best_num_valid = 0  # Prioritize more valid lanes
                        
                        for inv_idx in invalid_indices:
                            inv_seg = lane_segments[inv_idx]
                            
                            # Option A: Skip the right cluster
                            if inv_idx + 1 < num_c - 1:
                                c_left = inv_seg['left_cluster']
                                c_right = lane_segments[inv_idx + 1]['right_cluster']
                                width_a = abs(c_left['c0_corr'] - c_right['c0_corr'])
                                
                                temp_segments = []
                                for i, seg in enumerate(lane_segments):
                                    if i == inv_idx or i == inv_idx + 1:
                                        continue
                                    temp_segments.append(seg.copy())
                                
                                new_seg = {
                                    'left_cluster': c_left,
                                    'right_cluster': c_right,
                                    'width': width_a,
                                    'type': "ANCHO_4.2" if 3.5 < width_a <= 5.2 else ("ESTANDAR_2.8" if 2.3 <= width_a <= 3.5 else "INVALIDO"),
                                    'belongs_to': "5%" if 3.5 < width_a <= 5.2 else ("95%" if 2.3 <= width_a <= 3.5 else None)
                                }
                                temp_segments.append(new_seg)
                                temp_segments.sort(key=lambda x: x['left_cluster']['c0_corr'], reverse=True)
                                
                                scene_type, consistency = evaluate_scene(temp_segments)
                                
                                # Count how many valid lanes this scene has.
                                num_valid = sum(1 for seg in temp_segments if seg.get('type') != "INVALIDO" and seg.get('belongs_to') is not None)
                                
                                # Calculate score
                                if scene_type:
                                    score = sum([abs(seg['width'] - (IDEAL_28 if scene_type == "95%" else IDEAL_42)) 
                                                for seg in temp_segments if seg.get('type') != "INVALIDO"])
                                else:
                                    score = float('inf')
                                
                                # Prioritize: more valid lanes > better score
                                if consistency == "CONSISTENTE" and scene_type:
                                    if num_valid > best_num_valid or (num_valid == best_num_valid and score < best_score):
                                        best_num_valid = num_valid
                                        best_score = score
                                        best_scene = temp_segments
                                        best_type = scene_type
                            
                            # Option B: Skip the left cluster
                            if inv_idx - 1 >= 0:
                                c_left = lane_segments[inv_idx - 1]['left_cluster']
                                c_right = inv_seg['right_cluster']
                                width_b = abs(c_left['c0_corr'] - c_right['c0_corr'])
                                
                                temp_segments = []
                                for i, seg in enumerate(lane_segments):
                                    if i == inv_idx - 1 or i == inv_idx:
                                        continue
                                    temp_segments.append(seg.copy())
                                
                                new_seg = {
                                    'left_cluster': c_left,
                                    'right_cluster': c_right,
                                    'width': width_b,
                                    'type': "ANCHO_4.2" if 3.5 < width_b <= 5.2 else ("ESTANDAR_2.8" if 2.3 <= width_b <= 3.5 else "INVALIDO"),
                                    'belongs_to': "5%" if 3.5 < width_b <= 5.2 else ("95%" if 2.3 <= width_b <= 3.5 else None)
                                }
                                temp_segments.append(new_seg)
                                temp_segments.sort(key=lambda x: x['left_cluster']['c0_corr'], reverse=True)
                                
                                scene_type, consistency = evaluate_scene(temp_segments)
                                
                                # Count how many valid lanes this scene has
                                num_valid = sum(1 for seg in temp_segments if seg.get('type') != "INVALIDO" and seg.get('belongs_to') is not None)
                                
                                # Calculate score
                                if scene_type:
                                    score = sum([abs(seg['width'] - (IDEAL_28 if scene_type == "95%" else IDEAL_42)) 
                                                for seg in temp_segments if seg.get('type') != "INVALIDO"])
                                else:
                                    score = float('inf')
                                
                                # Prioritize: more valid lanes > better score
                                if consistency == "CONSISTENTE" and scene_type:
                                    if num_valid > best_num_valid or (num_valid == best_num_valid and score < best_score):
                                        best_num_valid = num_valid
                                        best_score = score
                                        best_scene = temp_segments
                                        best_type = scene_type
                        
                        if best_scene:
                            self.estructure = 95 if best_type == "95%" else 5

                        else:
                            self.estructure = 0
                else:
                    # Only one valid lane; use that one as a reference.
                    if valid_segments:
                        seg = valid_segments[0]
                        scene_type = seg['belongs_to']
                        self.estructure = 95 if scene_type == "95%" else 5
                    else:
                        self.estructure = 0
            
            # CASE 2: 3 clusters
            elif num_c == 3:
                
                valid_segments = [seg for seg in lane_segments if seg['type'] != "INVALIDO"]
                
                if len(valid_segments) == 2:
                    types = [seg['belongs_to'] for seg in valid_segments]
                    if types[0] == types[1]:
                        scene_type = types[0]
                    
                        self.estructure = 95 if scene_type == "95%" else 5
                    else:
                        self.estructure = 0
                elif len(valid_segments) == 1:
                    seg = valid_segments[0]
                    scene_type = seg['belongs_to']
                    self.estructure = 95 if scene_type == "95%" else 5
                else:
                    self.estructure = 0
            
            # CASE 3: 5 or more clusters
            elif num_c >= 5:
                
                best_scene = None
                best_type = None
                best_score = float('inf')
                
                for combo_size in [3, 2]:
                    for combo in itertools.combinations(lane_segments, combo_size):
                        indices = sorted([seg['left_idx'] for seg in combo] + [seg['right_idx'] for seg in combo])
                        if max(indices) - min(indices) + 1 > len(set(indices)):
                            continue
                        
                        scene_type, consistency = evaluate_scene(combo)
                        
                        if consistency == "CONSISTENTE" and scene_type:
                            score = sum([abs(seg['width'] - (IDEAL_28 if scene_type == "95%" else IDEAL_42)) for seg in combo])
                            if score < best_score:
                                best_score = score
                                best_scene = combo
                                best_type = scene_type
                    
                    if best_scene:
                        break
                
                if best_scene:
                    self.estructure = 95 if best_type == "95%" else 5
                else:
                    self.estructure = 0
            
            # CASE 4: 2 clusters
            elif num_c == 2:
                
                seg = lane_segments[0]
                if seg['type'] != "INVALIDO":
                    self.estructure = 95 if seg['belongs_to'] == "95%" else 5
                else:
                    self.estructure = 0
        #========================================================================================================================================

        # Specific case of the entry
        if self.detection and self.alpha and aux is not None and (abs(aux['c0_corr']) < 0.5 or (aux['min_x'] > 10.0 and abs(aux['c0_corr']) < 1.5)) and self.estructure == 5:
            self.n = self.n + 1 if self.n < 15 else 15
        else:
            self.n = 0

        # 4. Message Construction and Visual Debugging
        out = LaneModel()
        out.header = msg.header

        if best_l and best_r:
            # Case 1: Both lines are detected
            c0_l = float(best_l['c0'] * np.cos(np.arctan(best_l['c1'])))
            c0_r = float(best_r['c0'] * np.cos(np.arctan(best_r['c1'])))
            out.lane_width = float(abs(c0_l - c0_r))
            out.d_lat = float((c0_l + c0_r) / 2.0)
            # We average the local angle and the curvature for the centerline.
            avg_yaw = (best_l['c1'] + best_r['c1']) / 2.0

            if best_l['c2'] != 0 and best_r['c2'] != 0:
                avg_curv = (best_l['c2'] + best_r['c2']) / 2.0
            elif best_l['c2'] != 0:
                avg_curv = best_l['c2']
            elif best_r['c2'] != 0:
                avg_curv = best_r['c2']
            else:
                avg_curv = 0.0

            out.curvature = float(2.0 * avg_curv)
            out.yaw = float(np.arctan(avg_yaw))
            out.confidence = 1.0
            
        elif best_l or best_r:
            # Case 2 and 3: Only one line is detected
            ref = best_l if best_l else best_r
            sign = -1 if best_l else 1
            
            out.lane_width = self.expected_width
            out.yaw = float(np.arctan(ref['c1']))
            out.curvature = float(2.0 * ref['c2']) 

            c0_ref = float(ref['c0'] * np.cos(np.arctan(ref['c1'])))
            if not best_l and best_r and abs(c0_ref) > self.expected_width: 
                # Case 3: We detect only one edge and force an offset of 4.5 m.
                distancia_al_centro = (3.0*self.expected_width)/2.0
                out.d_lat = float(c0_ref + distancia_al_centro)
                out.confidence = 0.5
            elif best_l and not best_r and abs(c0_ref) > self.expected_width: 
                # Case 3: We detect only one edge and force an offset of 4.5 m.
                distancia_al_centro = (3.0*self.expected_width)/2.0
                out.d_lat = float(c0_ref - distancia_al_centro)
                out.confidence = 0.5
            else:
                # Case 2: We detect only one lane line and force a 1.4m offset.
                out.d_lat = float(c0_ref + sign * (self.expected_width / 2.0))
                out.confidence = 0.7
            
        else:
            out.lane_width = self.expected_width
            out.d_lat = float(0.0)
            out.yaw = float(0.0)
            out.curvature = float(0.0)
            out.confidence = float(0.0)
            #add_curve_to_debug(0.0, 0.0, 0.0, c_center)

        # Case 4: Entrance requiring traffic to always keep right
        if self.n >= 1 and (out.d_lat > 0 or abs(out.d_lat - self.d_lat_last) > self.max_d_lat_change ):
            if aux is not None:
                out.d_lat = aux['c0_corr'] - 1.0 
                self.d_lat_last = 10000
                out.confidence = 0.4

        # 5. Lane Skip and Rescan Validation
        if ( abs(out.d_lat - self.d_lat_last) > self.max_d_lat_change or out.lane_width < 1.0 ) and self.d_lat_last != 10000:
            recuperado = False

            # Recovery attempt 1: What if we only use the left line?
            if best_l:
                c0_l = float(best_l['c0'] * np.cos(np.arctan(best_l['c1'])))
                d_lat_solo_l = float(c0_l + (self.expected_width / 2.0))
                if abs(d_lat_solo_l - self.d_lat_last) < self.max_d_lat_change:
                    out.lane_width = self.expected_width
                    out.yaw = float(np.arctan(best_l['c1']))
                    out.curvature = float(2.0 * best_l['c2']) 
                    out.d_lat = d_lat_solo_l
                    out.confidence = 0.3  
                    recuperado = True
                    best_r = best_l
                    best_l = None
            
            # Recovery attempt 2: What if we only use the right line?
            if best_r:
                c0_r = float(best_r['c0'] * np.cos(np.arctan(best_r['c1'])))
                d_lat_solo_r = float(c0_r - (self.expected_width / 2.0))
                if abs(d_lat_solo_r - self.d_lat_last) < self.max_d_lat_change:
                    out.lane_width = self.expected_width
                    out.yaw = float(np.arctan(best_r['c1']))
                    out.curvature = float(2.0 * best_r['c2']) 
                    out.d_lat = d_lat_solo_r
                    out.confidence = 0.3
                    best_l = best_r
                    best_r = None
                    recuperado = True

            # If a jump persists after attempting recovery, then we invalidate it.
            if not recuperado:                
                out.confidence = 0.0
                out.d_lat = self.d_lat_last
                out.yaw = 0.0
                out.curvature = 0.0
        
        self.d_lat_last = out.d_lat if out.confidence > 0 else self.d_lat_last
        self.pub_model.publish(out)

        # =========================================================================
        # --- CONSTRUCTION OF THE AUXILIARY MODEL ---
        # =========================================================================        
        left_lane = LaneModel()
        left_lane.header = msg.header
        right_lane = LaneModel()
        right_lane.header = msg.header 
        other_l_cands = []
        other_r_cands = []

        other_l_cands = [c for c in l_cands if c['cluster_id'] != best_l['cluster_id']] if best_l else []
        other_r_cands = [c for c in r_cands if c['cluster_id'] != best_r['cluster_id']] if best_r else []

        # =========================================================================
        # --- FILTERING AND LOOP FOR THE LEFT LANE ---
        # =========================================================================
        left_lane_found = False
        
        if other_l_cands:
            # We arrange the candidates from closest to farthest from the original center.
            sorted_l_cands = sorted(other_l_cands, key=lambda x: abs(x['mean']))
            
            for next_l in sorted_l_cands:
                calculated_width = float(abs(best_l['c0_corr'] - next_l['c0_corr']))
                calculate_d_lat = float((best_l['c0_corr'] + next_l['c0_corr']) / 2.0)
                
                # We verify whether the measured width falls within the acceptable range.
                if 1.7 <= calculated_width <= 3.5:
                    left_lane.lane_width = calculated_width
                    left_lane.d_lat = calculate_d_lat
                    
                    avg_yaw_left = (best_l['c1'] + next_l['c1']) / 2.0
                    avg_curv_left = (best_l['c2'] + next_l['c2']) / 2.0 if (best_l['c2'] != 0 and next_l['c2'] != 0) else (best_l['c2'] if best_l['c2'] != 0 else next_l['c2'])
                    
                    left_lane.curvature = float(2.0 * avg_curv_left)
                    left_lane.yaw = float(np.arctan(avg_yaw_left))
                    left_lane.confidence = 1.0
                    
                    left_lane_found = True
                    break  # Found; we exit the iterative loop.
            
        # =========================================================================
        # --- FILTERING AND LOOP FOR THE RIGHT LANE--
        # =========================================================================
        right_lane_found = False
        
        if other_r_cands:
            # We arrange the candidates from closest to farthest from the original center.
            sorted_r_cands = sorted(other_r_cands, key=lambda x: abs(x['mean']))
            
            for next_r in sorted_r_cands:
                calculated_width = float(abs(best_r['c0_corr'] - next_r['c0_corr']))
                calculate_d_lat = float((best_r['c0_corr'] + next_r['c0_corr']) / 2.0)
                
                # We verify whether the measured width falls within the acceptable range.
                if 1.7 <= calculated_width <= 3.5:
                    right_lane.lane_width = calculated_width
                    right_lane.d_lat = calculate_d_lat
                    
                    avg_yaw_right = (best_r['c1'] + next_r['c1']) / 2.0                    
                    avg_curv_right = (best_r['c2'] + next_r['c2']) / 2.0 if (best_r['c2'] != 0 and next_r['c2'] != 0) else (best_r['c2'] if best_r['c2'] != 0 else next_r['c2'])
                    
                    right_lane.curvature = float(2.0 * avg_curv_right)
                    right_lane.yaw = float(np.arctan(avg_yaw_right))
                    right_lane.confidence = 1.0
                    
                    right_lane_found = True
                    break  # Found; we exit the iterative loop.

        # =========================================================================
        # --- Y-INTERSECTION EVALUATION OF OBJECTS USING THE RAW MODEL ---
        # =========================================================================
        # Auxiliary function to evaluate intersection with any lane.
        def verificar_interseccion(lane, x_min=0.1, x_max=25.0):
            if lane.confidence <= 0.0:
                return False, 0.0, 0.0
            c2, tan_yaw, half_w = 0.5 * lane.curvature, np.tan(lane.yaw), lane.lane_width / 2.0
            for det in vehiculos:
                x_eval = float((det.x1 + det.x2) / 2.0)
                if x_min < x_eval <= x_max:
                    y_center = c2 * (x_eval**2) + tan_yaw * x_eval + lane.d_lat
                    l_min, l_max = min(y_center - half_w, y_center + half_w), max(y_center - half_w, y_center + half_w)
                    o_min, o_max = min(det.y1, det.y2), max(det.y1, det.y2)
                    if max(l_min, o_min) <= min(l_max, o_max): # Hay solapamiento
                        metros_inv = min(l_max, o_max) - max(l_min, o_min)
                        return True, metros_inv / lane.lane_width, x_eval
            return False, 0.0, 0.0

        # Function to invalidate and publish a lane to zero.
        def anular_carril(lane, publisher, attr_name):
            lane.lane_width = lane.d_lat = lane.curvature = lane.yaw = lane.confidence = 0.0
            publisher.publish(lane)
            setattr(self, attr_name, None)

        if out.confidence > 0.0 and len(self.detections_meter_list) > 0 and (self.change_left or self.change_right):
            clases_vehiculos = {0, 1, 2, 3, 4, 5}
            vehiculos = [d for d in self.detections_meter_list if d.class_id in clases_vehiculos]

            if self.change_left:
                _, izq_porc, _ = verificar_interseccion(left_lane) if left_lane_found else (True, 0.0, 0.0)
                
                if left_lane_found and izq_porc < 0.40:
                    self.pub_model_left.publish(left_lane)
                    self.d_lat_left = left_lane.d_lat
                    anular_carril(right_lane, self.pub_model_right, 'd_lat_right')
                    self.change_n = self.change_n + 1
                elif left_lane_found and izq_porc >= 0.40:
                    self.change_n = 0

            if self.change_right:
                _, der_porc, _ = verificar_interseccion(right_lane) if right_lane_found else (True, 0.0, 0.0)
                
                if right_lane_found and der_porc < 0.40:
                    self.pub_model_right.publish(right_lane)
                    self.d_lat_right = right_lane.d_lat                    
                    anular_carril(left_lane, self.pub_model_left, 'd_lat_left')
                    self.change_n = self.change_n + 1
                elif right_lane_found and der_porc >= 0.40:
                    self.change_n = 0
        
        elif out.confidence > 0.0 and len(self.detections_meter_list) == 0 and (self.change_left or self.change_right):
            if self.change_left and left_lane_found:
                self.pub_model_left.publish(left_lane)
                self.d_lat_left = left_lane.d_lat
                anular_carril(right_lane, self.pub_model_right, 'd_lat_right')
                self.change_n = self.change_n + 1

            if self.change_right and right_lane_found:             
                self.pub_model_right.publish(right_lane)
                self.d_lat_right = right_lane.d_lat
                anular_carril(left_lane, self.pub_model_left, 'd_lat_left')
                self.change_n = self.change_n + 1
        else:
            anular_carril(left_lane, self.pub_model_left, 'd_lat_left')
            anular_carril(right_lane, self.pub_model_right, 'd_lat_right')

        # Lane change
        if self.change_left and self.d_lat_left is not None and self.change_n  >= 5:
            self.d_lat_last = self.d_lat_left
            self.change_left = False
            self.change_n = 0
        
        if self.change_right and self.d_lat_right is not None and self.change_n >= 5:
            self.d_lat_last = self.d_lat_right
            self.change_right = False
            self.change_n = 0

def main(args=None):
    rclpy.init(args=args)
    node = QuadraticLaneEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()