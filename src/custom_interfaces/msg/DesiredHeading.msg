HelperHeading heading
uint8 steering
# sail indicates whether local_pathfinding is producing a valid heading:
#   - sail=True:  local_pathfinding is working; heading is valid.
#   - sail=False: local_pathfinding has failed, so heading is set to 0.0 and
#                 should be ignored. The controller and network systems must
#                 handle this failure case.
bool sail
