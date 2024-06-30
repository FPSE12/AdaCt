# AdaCt
lidar_odometry: ikdtree_for map manager 

lidar_odometry_downsample: downsample the oricloud

lidar_odometry_new: ikdtree use like fastlio2


##record

featureMap better than  featureMapNew


## note
use the c++ 03  opt the computation

downsample: map->big(to find more points), curr->frame->big


a2d:
indoor ave: 0.4-0.45: people  0.3-0.4
55lou ave:  indoor 0.45-0.6

outdoor sipailou: 0.4 -0.52

forest: downsample most is ground, 树干上的点大多邻居不够

multiNode&&resolution: always update, liwenzhenghuan is also wrong
                        but indoor is better than keypose_update

2-1:
multiresolution :  better than else;
mutlmode : can work


three node : 体素地图效果好一点，多分辨率实时性更高效果差一点（人的影响）


