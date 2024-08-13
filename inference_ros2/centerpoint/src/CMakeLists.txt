add_library(centerpoint SHARED
  centerpoint/centerpoint.cc
  centerpoint/network.cc
  centerpoint/voxelization.cu
  centerpoint/voxelization.cc
  centerpoint/postprocess.cu
  centerpoint/postprocess.cc
  pillarscatter/pillarscatter-kernel.cu
  pillarscatter/pillarscatter.cc
)

target_link_libraries(centerpoint
  ${THIRD_PARTY_LIBS})
