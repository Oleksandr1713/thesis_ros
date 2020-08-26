#include "ros/ros.h"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/GetMap.h"
#include "simulation/AddSignOnMapMsg.h"
#include "simulation/RemoveSignFromMapMsg.h"
#include "my_lib/auxiliary_func.h"

using namespace auxiliary_func;

grid_map::GridMap *map;
ros::Publisher pub_map;
ros::Publisher pub_signs;

const std::string ORIGIN_LAYER = "origin_map_layer";
const std::string AUGMENTED_LAYER = "augmented_map_layer";


/**
 * Method calculates the dimension of a layer and describes it as two diagonal points of a rectangle.
 *
 * @param layers A data structure containing a list of layers of a navigation environment.
 * @param layer_name The name of desired layer, that is needed to work.
 * @param bottom_left_point Ref to the undefined bottom-left coordinate of the layer in the rviz coordinate system.
 * @param top_right_point Ref to the undefined top-right coordinate of the layer in the rviz coordinate system.
 * @return TRUE if a layer exists, or FALSE otherwise.
 */
bool findLayerDimensionCoordinates(const grid_map::GridMap* layers, const std::string layer_name, grid_map::Position& bottom_left_point, grid_map::Position& top_right_point){
    if (layers->exists(layer_name)){
        grid_map::Position center = layers->getPosition();
        float map_half_width = layers->getLength().x() / 2;
        float map_half_height = layers->getLength().y() / 2;

        bottom_left_point.x() = center.x() - map_half_width;
        bottom_left_point.y() = center.y() - map_half_height;
        top_right_point.x() = center.x() + map_half_width;
        top_right_point.y() = center.y() + map_half_height;
        return true;
    } else{
        return false;
    }
}

/**
 * Method calculates the intersection points of the line with 2 edges of the layer.
 * The line is described as a linear equation based on 2 input points of the method.
 *
 * @param firstPoint The first point to describe the line.
 * @param secondPoint The second point to describe the line.
 * @param layers A data structure containing a list of layers of a navigation environment.
 * @param layer_name The name of desired layer, that is needed to work.
 * @param coordLeftEdgeInter Ref to the undefined coordinate of the intersection of the line with the left layer edge. If
 * the linear equation is a vertical line, then this coordinate represents an intersection with a bottom edge of the layer.
 * @param coordRightEdgeInter Ref to the undefined coordinate of the intersection of the line with the right layer edge. If
 * the linear equation is a vertical line, then this coordinate represents an intersection with a top edge of the layer.
 * @return TRUE if a layer and intersection points exist, or FALSE otherwise.
 */
bool findIntersectionsOfLineWithHeightEdgesOfMapFrame(grid_map::Position& firstPoint, grid_map::Position& secondPoint,
                                                        const grid_map::GridMap* layers, const std::string layer_name,
                                                        grid_map::Position& coordLeftEdgeInter, grid_map::Position& coordRightEdgeInter){
    grid_map::Position bottom_left;
    grid_map::Position top_right;
    bool dimension_exist = findLayerDimensionCoordinates(layers, layer_name, bottom_left, top_right);

    if(dimension_exist){
        if(firstPoint.x() == secondPoint.x()){
            /*This is the case when the line is vertical, therefore it will never cross any other vertical line or
             * right/left edge of the layer. However, 2 intersection points of the line with horizontal edges (bottom
             * and top) of the layer can be defined.*/
            coordLeftEdgeInter.x() = firstPoint.x();
            coordLeftEdgeInter.y() = bottom_left.y();
            coordRightEdgeInter.x() = firstPoint.x();
            coordRightEdgeInter.y() = top_right.y();
        } else{
            float slope = (firstPoint.y() - secondPoint.y())/(firstPoint.x() - secondPoint.x());

            coordLeftEdgeInter.x() = bottom_left.x();
            coordLeftEdgeInter.y() = slope * coordLeftEdgeInter.x() - (slope * firstPoint.x() - firstPoint.y());

            coordRightEdgeInter.x() = top_right.x();
            coordRightEdgeInter.y() = slope * coordRightEdgeInter.x() - (slope * secondPoint.x() - secondPoint.y());
        }
    }
    return dimension_exist;
}

/**
 * Method defines the coordinates of a sign footprint and displays this footprint on the layer as a line drawing.
 *
 * @param layers A data structure containing a list of layers of a navigation environment.
 * @param reference_layer_name The name of layer, that is not modified and contains only information of the original
 * static map.
 * @param alterable_layer_name The name of layer, that includes information about sign footprints.
 * @param signLeftEdgeCoordinate The coordinate point of the left edge of the sign.
 * @param signRightEdgeCoordinate The coordinate point of the right edge of the sign.
 * @param footprintFirstCoordinate Ref to the first undefined coordinate, that describes a sign footprint.
 * @param footprintSecondCoordinate Ref to the second undefined coordinate, that describes a sign footprint.
 * @return TRUE if the sign footprint has been drawn on the map successfully, or FALSE otherwise.
 */
bool drawSignFootprint(grid_map::GridMap* layers, const std::string reference_layer_name, const std::string alterable_layer_name,
                        grid_map::Position& signLeftEdgeCoordinate, grid_map::Position& signRightEdgeCoordinate,
                        grid_map::Position& footprintFirstCoordinate, grid_map::Position& footprintSecondCoordinate){

    grid_map::Position coordLeftEdgeInter;
    grid_map::Position coordRightEdgeInter;
    bool success = findIntersectionsOfLineWithHeightEdgesOfMapFrame(signLeftEdgeCoordinate, signRightEdgeCoordinate, layers, alterable_layer_name, coordLeftEdgeInter, coordRightEdgeInter);

    if(success){
        for (grid_map::LineIterator iterator(*layers, signLeftEdgeCoordinate, coordLeftEdgeInter); !iterator.isPastEnd(); ++iterator) {
            if(layers->at(reference_layer_name, *iterator) > 0.8){
                break;
            }
            layers->getPosition(*iterator, footprintFirstCoordinate);
        }
        for (grid_map::LineIterator iterator(*layers, signLeftEdgeCoordinate, coordRightEdgeInter); !iterator.isPastEnd(); ++iterator) {
            if(layers->at(reference_layer_name, *iterator) > 0.8){
                break;
            }
            layers->getPosition(*iterator, footprintSecondCoordinate);
        }
        footprintFirstCoordinate.x() = roundToNearest(footprintFirstCoordinate.x(), 3);
        footprintFirstCoordinate.y() = roundToNearest(footprintFirstCoordinate.y(), 3);
        footprintSecondCoordinate.x() = roundToNearest(footprintSecondCoordinate.x(), 3);
        footprintSecondCoordinate.y() = roundToNearest(footprintSecondCoordinate.y(), 3);

        for (grid_map::LineIterator iterator(*layers, footprintFirstCoordinate, footprintSecondCoordinate); !iterator.isPastEnd(); ++iterator) {
            layers->at(alterable_layer_name, *iterator) = 1.0;
        }
    }
    return success;
}

/**
 * Method erases a sign footprint from the map based on 2 coordinates that describe this footprint.
 *
 * @param layers A data structure containing a list of layers of a navigation environment.
 * @param alterable_layer_name The name of layer, that includes information about sign footprints.
 * @param footprintFirstCoordinate The first coordinate point that describes the linear equation of the sign footprint.
 * @param footprintSecondCoordinate The second coordinate point that describes the linear equation of the sign footprint.
 * @return TRUE if a layer exists, or FALSE otherwise.
 */
bool eraseSignFootprint(grid_map::GridMap* layers, const std::string alterable_layer_name,
                        grid_map::Position& footprintFirstCoordinate, grid_map::Position& footprintSecondCoordinate){
    if (layers->exists(alterable_layer_name)){
        for (grid_map::LineIterator iterator(*layers, footprintFirstCoordinate, footprintSecondCoordinate); !iterator.isPastEnd(); ++iterator) {
            layers->at(alterable_layer_name, *iterator) = 0.0;
        }
        return true;
    } else{
        return false;
    }
}

void drawPoint(grid_map::GridMap* layers, const std::string layer_name, grid_map::Position& point){
    for (grid_map::CircleIterator it(*layers, point, 0.05); !it.isPastEnd(); ++it) {
        layers->at(layer_name, *it) = 1;
    }
}

bool addSignOnMap(simulation::AddSignOnMapMsg::Request &request, simulation::AddSignOnMapMsg::Response &response){
    grid_map::Position sign_point_1(request.x1_coordinate, request.y1_coordinate);
    grid_map::Position sign_point_2(request.x2_coordinate, request.y2_coordinate);

    grid_map::Position footprint_point_1;
    grid_map::Position footprint_point_2;
    bool success = drawSignFootprint(map, ORIGIN_LAYER, AUGMENTED_LAYER, sign_point_1, sign_point_2, footprint_point_1, footprint_point_2);

    response.success = success;
    if(success) {
        response.x1_intersection = footprint_point_1.x();
        response.y1_intersection = footprint_point_1.y();
        response.x2_intersection = footprint_point_2.x();
        response.y2_intersection = footprint_point_2.y();

        nav_msgs::OccupancyGrid augmentedMapOccupancyGrid;
        grid_map::GridMapRosConverter::toOccupancyGrid(*map, AUGMENTED_LAYER,0.0, 1.0, augmentedMapOccupancyGrid);

        pub_map.publish(augmentedMapOccupancyGrid);
    }
    return success;
}

bool removeSignFromMap(simulation::RemoveSignFromMapMsg::Request &request, simulation::RemoveSignFromMapMsg::Response &response){
    grid_map::Position footprint_point_1(request.x1_intersection, request.y1_intersection);
    grid_map::Position footprint_point_2(request.x2_intersection, request.y2_intersection);

    bool success = eraseSignFootprint(map, AUGMENTED_LAYER, footprint_point_1, footprint_point_2);

    response.success = success;
    if(success){
        nav_msgs::OccupancyGrid augmentedMapOccupancyGrid;
        grid_map::GridMapRosConverter::toOccupancyGrid(*map, AUGMENTED_LAYER,0.0, 1.0, augmentedMapOccupancyGrid);

        pub_map.publish(augmentedMapOccupancyGrid);
    }
    return true;
}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "map_enrichment_node");
    ros::NodeHandle nh("~");
    ros::ServiceClient client_map_server = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    ros::ServiceServer srv_add_sign = nh.advertiseService("add_sign_on_map", addSignOnMap);
    ros::ServiceServer srv_remove_sign = nh.advertiseService("remove_sign_from_map", removeSignFromMap);
    pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/augmented_map", 1, true);

    nav_msgs::GetMap srv;
    client_map_server.call(srv);
    nav_msgs::OccupancyGrid *pStaticMapOccupancyGrid = &srv.response.map;
    pub_map.publish(*pStaticMapOccupancyGrid);

    map = new grid_map::GridMap({ORIGIN_LAYER, AUGMENTED_LAYER});
    grid_map::GridMapRosConverter::fromOccupancyGrid(*pStaticMapOccupancyGrid, ORIGIN_LAYER, *map);
    grid_map::GridMapRosConverter::fromOccupancyGrid(*pStaticMapOccupancyGrid, AUGMENTED_LAYER, *map);
    map->setFrameId(pStaticMapOccupancyGrid->header.frame_id);

    ROS_INFO("Map grid with size %.2f x %.2f m (%i x %i cells).",
             map->getLength().x(), map->getLength().y(), map->getSize()(0), map->getSize()(1));
    ROS_INFO("Map grid with (x=%f, y=%f) center coordinates.", map->getPosition()(0), map->getPosition()(1));
    ROS_INFO("Map Enrichment node is up.");

    ros::spin();
    return 0;
}
