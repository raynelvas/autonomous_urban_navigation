# segmentation_msgs/msg/SegmentationData.msg
std_msgs/Header header

# Matriz 2D de clases (comprimida o flat array)
uint8[] mask_data
uint32 height
uint32 width
float32[] probabilities  # opcional: confianzas
