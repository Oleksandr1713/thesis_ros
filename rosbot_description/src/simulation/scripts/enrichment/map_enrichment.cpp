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
ros::Publisher pub_map_update;

/* Layer names */
const std::string ORIGIN_LAYER = "origin_map_layer";
const std::string AUGMENTED_LAYER = "augmented_map_layer";

/* Topic names */
const std::string PUB_AUGMENTED_MAP = "/augmented_map";                     // topic to publish the updated map via OccupancyGrid message

/* Service names */
const std::string REQ_ORIGINAL_MAP = "/static_map";                         // service name, where the original static map can be requested
const std::string ADV_ENRICH_AUGMENTED_MAP = "add_sign_on_map";             // name of advertised service that can be requested by other nodes to add a sign on the map
const std::string ADV_IMPOVERISH_AUGMENTED_MAP = "remove_sign_from_map";    // name of advertised service that can be requested by other nodes to remove a sign from the map

const double UNIT_RADIUS = 0.05;    // meters
const double AZIMUTH = 95;          // degrees


bool isAzimuthValid(double azimuth){
    return (0 < std::abs (azimuth) && std::abs (azimuth) < 180);
}

double getDegrees(double radians){
    double degrees = radians * 180/M_PI;
    while(degrees < 0){
        degrees = 360 + degrees;
    }
    while(degrees > 360){
        degrees = degrees - 360;
    }
    return degrees;
}

double getRadians(double degrees){
    return degrees * M_PI/180;
}

void getReferenceAzimuthPoint(grid_map::Position& referenceOriginPoint, double referenceAzimuth, grid_map::Position& referenceAzimuthPoint){
    referenceAzimuthPoint.x() = referenceOriginPoint.x() + UNIT_RADIUS * cos(getRadians(referenceAzimuth));
    referenceAzimuthPoint.y() = referenceOriginPoint.y() + UNIT_RADIUS * sin(getRadians(referenceAzimuth));
}

/**
 * Method calculates the dimension of a layer and describes it as two diagonal points of a rectangle.
 *
 * @param layers A data structure containing a list of layers of a navigation environment.
 * @param layer_name The name of desired layer, that is needed to work.
 * @param bottom_left_point Ref to the undefined bottom-left coordinate of the layer in the rviz coordinate system.
 * @param top_right_point Ref to the undefined top-right coordinate of the layer in the rviz coordinate system.
 * @return TRUE if a layer exists, or FALSE otherwise.
 */
bool findLayerDimensionCoordinates(const grid_map::GridMap* layers, const std::string& layer_name, grid_map::Position& bottom_left_point, grid_map::Position& top_right_point){
    if (layers->exists(layer_name)){
        grid_map::Position center = layers->getPosition();
        double map_half_width = layers->getLength().x() / 2;
        double map_half_height = layers->getLength().y() / 2;

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
 * @param bottom_left_of_layer The bottom-left coordinate of the layer in the rviz coordinate system.
 * @param top_right_of_layer The top-right coordinate of the layer in the rviz coordinate system.
 * @param firstPoint The first point to describe the line.
 * @param secondPoint The second point to describe the line.
 * @param intersection_1 Ref to the undefined coordinate of the intersection of the line with a layer edge. If
 * the linear equation is a vertical line, then this coordinate represents an intersection with a bottom edge of the layer.
 * @param intersection_2 Ref to the undefined coordinate of the intersection of the line with a layer edge. If
 * the linear equation is a vertical line, then this coordinate represents an intersection with a top edge of the layer.
 * @return
 */
void findIntersectionsOfLineWithLayerEdgesOfMapFrame(grid_map::Position& bottom_left_of_layer, grid_map::Position& top_right_of_layer,
                                                      grid_map::Position& firstPoint, grid_map::Position& secondPoint,
                                                      grid_map::Position& intersection_1, grid_map::Position& intersection_2){
    if(firstPoint.x() == secondPoint.x()){
        /*This is the case when the line is vertical, therefore it will never cross any other vertical line or
         * right/left edge of the layer. However, 2 intersection points of the line with horizontal edges (bottom
         * and top) of the layer can be defined.*/
        intersection_1.x() = firstPoint.x();
        intersection_1.y() = bottom_left_of_layer.y();
        intersection_2.x() = firstPoint.x();
        intersection_2.y() = top_right_of_layer.y();
    } else{
        double slope = (firstPoint.y() - secondPoint.y())/(firstPoint.x() - secondPoint.x());

        intersection_1.x() = bottom_left_of_layer.x();
        intersection_1.y() = slope * intersection_1.x() - (slope * firstPoint.x() - firstPoint.y());

        intersection_2.x() = top_right_of_layer.x();
        intersection_2.y() = slope * intersection_2.x() - (slope * secondPoint.x() - secondPoint.y());
    }
}

/**
 * Method calculates the intersection point of the line with a single edge of the layer. The line is described as
 * a linear equation obtained based on a coordinate of a sign origin, a sign direction and an azimuth angle, which is
 * calculated relatively to the sign direction and describes a direction where the intersection must be found.
 *
 * @param layers A data structure containing a list of layers of a navigation environment.
 * @param alterable_layer_name The name of layer, that includes information about sign footprints.
 * @param sign_point The center coordinate of the sign : (x, y).
 * @param sign_direction The angle in degrees between the x' axis (a perpendicular to a surface of a sign) and x axis (a part of
 * a Cartesian reference system of a layer frame).
 * @param edge_azimuth The relative shift in degrees to define the direction in which the intersection point must be
 * found.
 * @param intersection_point Ref to the undefined coordinate of the intersection of the line with a layer edge.
 * @return TRUE if everything goes well, or FALSE otherwise.
 */
bool findIntersectionPointWithLayerEdge(grid_map::GridMap* layers, const std::string& alterable_layer_name,
                                     grid_map::Position& sign_point, double sign_direction, double edge_azimuth, grid_map::Position& intersection_point){
    if(!isAzimuthValid(edge_azimuth)){
        return false;
    }

    grid_map::Position bottom_left;
    grid_map::Position top_right;
    bool dimension_exist = findLayerDimensionCoordinates(layers, alterable_layer_name, bottom_left, top_right);
    if(!dimension_exist){
        return false;
    }

    grid_map::Position ref_azimuth_point;
    getReferenceAzimuthPoint(sign_point, sign_direction + edge_azimuth, ref_azimuth_point);

    grid_map::Position intersection_1;
    grid_map::Position intersection_2;
    findIntersectionsOfLineWithLayerEdgesOfMapFrame(bottom_left, top_right, sign_point, ref_azimuth_point, intersection_1, intersection_2);

    if(((intersection_1.x() <=  ref_azimuth_point.x() && ref_azimuth_point.x() <= sign_point.x()) || (sign_point.x() <= ref_azimuth_point.x() && ref_azimuth_point.x() <= intersection_1.x()))
        && ((intersection_1.y() <=  ref_azimuth_point.y() && ref_azimuth_point.y() <= sign_point.y()) || (sign_point.y() <= ref_azimuth_point.y() && ref_azimuth_point.y() <= intersection_1.y()))){
        intersection_point.x() = intersection_1.x();
        intersection_point.y() = intersection_1.y();
    } else{
        intersection_point.x() = intersection_2.x();
        intersection_point.y() = intersection_2.y();
    }
    return true;
}

void drawLine(grid_map::GridMap* layers, const std::string& layer_name, grid_map::Position& point_1, grid_map::Position& point_2, float cell_value = 1.0){
    for (grid_map::LineIterator iterator(*layers, point_1, point_2); !iterator.isPastEnd(); ++iterator) {
        layers->at(layer_name, *iterator) = cell_value;
    }
}

void drawPoint(grid_map::GridMap* layers, const std::string& layer_name, grid_map::Position& point, float cell_value = 1.0){
    for (grid_map::CircleIterator it(*layers, point, 0.05); !it.isPastEnd(); ++it) {
        layers->at(layer_name, *it) = cell_value;
    }
}

void eraseLine(grid_map::GridMap* layers, const std::string& layer_name, grid_map::Position& point_1, grid_map::Position& point_2){
    drawLine(layers, layer_name, point_1, point_2, 0.0);
}

void erasePoint(grid_map::GridMap* layers, const std::string& layer_name, grid_map::Position& point){
    drawPoint(layers, layer_name, point, 0.0);
}

/**
 * Method draws a full sign footprint on the layer. The full footprint line is a connection of 3 point:
 * footprint_point_1 with sign_coordinate and sign_coordinate with footprint_point_2.
 *
 * @param layers A data structure containing a list of layers of a navigation environment.
 * @param reference_layer_name The name of layer, that is not modified and contains only information of the original
 * static map.
 * @param alterable_layer_name The name of layer, that includes information about sign footprints.
 * @param sign_coordinate The center coordinate of the sign.
 * @param edge_intersection_1 The coordinate of the intersection of the line with a layer edge.
 * @param edge_intersection_2 The coordinate of the intersection of the line with a layer edge.
 * @param footprint_point_1 Ref to the first undefined coordinate, that describes an intersection point of a sign
 * footprint with a nearest wall.
 * @param footprint_point_2 Ref to the second undefined coordinate, that describes an intersection point of a sign
 * footprint with a nearest wall.
 * @return
 */
void drawSignFootprintFromSignOriginToNearestWalls(grid_map::GridMap* layers, const std::string& reference_layer_name, const std::string& alterable_layer_name,
                                                   grid_map::Position& sign_coordinate,
                                                   grid_map::Position& edge_intersection_1, grid_map::Position& edge_intersection_2,
                                                   grid_map::Position& footprint_point_1, grid_map::Position& footprint_point_2){

    for (grid_map::LineIterator iterator(*layers, sign_coordinate, edge_intersection_1); !iterator.isPastEnd(); ++iterator) {
        if(layers->at(reference_layer_name, *iterator) > 0.8){
            break;
        }
        layers->getPosition(*iterator, footprint_point_1);
    }
    for (grid_map::LineIterator iterator(*layers, sign_coordinate, edge_intersection_2); !iterator.isPastEnd(); ++iterator) {
        if(layers->at(reference_layer_name, *iterator) > 0.8){
            break;
        }
        layers->getPosition(*iterator, footprint_point_2);
    }
    footprint_point_1.x() = roundToNearest(footprint_point_1.x(), 3);
    footprint_point_1.y() = roundToNearest(footprint_point_1.y(), 3);
    footprint_point_2.x() = roundToNearest(footprint_point_2.x(), 3);
    footprint_point_2.y() = roundToNearest(footprint_point_2.y(), 3);

    drawLine(layers, alterable_layer_name, sign_coordinate, footprint_point_1);
    drawLine(layers, alterable_layer_name, sign_coordinate, footprint_point_2);
    drawPoint(layers, alterable_layer_name, sign_coordinate);
}

/**
 * Method erases a sign footprint from the map based on 2 coordinates that describe this footprint.
 *
 * @param layers A data structure containing a list of layers of a navigation environment.
 * @param alterable_layer_name The name of layer, that includes information about sign footprints.
 * @param sign_center The center coordinate of sign.
 * @param footprint_point_1 The first coordinate that represents an intersection point of the sign footprint with
 * a nearest wall.
 * @param footprint_point_2 The second coordinate that represents an intersection point of the sign footprint with
 * a nearest wall.
 * @return TRUE if a layer exists, or FALSE otherwise.
 */
bool eraseSignFootprint(grid_map::GridMap* layers, const std::string& alterable_layer_name,
                        grid_map::Position& sign_center,
                        grid_map::Position& footprint_point_1, grid_map::Position& footprint_point_2){
    if (layers->exists(alterable_layer_name)){
        eraseLine(layers, alterable_layer_name, sign_center, footprint_point_1);
        eraseLine(layers, alterable_layer_name, sign_center, footprint_point_2);
        erasePoint(layers, alterable_layer_name, sign_center);
        return true;
    } else{
        return false;
    }
}

bool addSignOnMap(simulation::AddSignOnMapMsg::Request &request, simulation::AddSignOnMapMsg::Response &response){
    grid_map::Position sign_center(request.x_center, request.y_center);
    double sign_direction = getDegrees(request.dir_radians); // degrees

    grid_map::Position edge_intersection_1;
    grid_map::Position edge_intersection_2;
    bool success_1 = findIntersectionPointWithLayerEdge(map, AUGMENTED_LAYER, sign_center, sign_direction, AZIMUTH, edge_intersection_1);
    bool success_2 = findIntersectionPointWithLayerEdge(map, AUGMENTED_LAYER, sign_center, sign_direction, -AZIMUTH, edge_intersection_2);

    if(success_1 && success_2) {
        grid_map::Position footprint_point_1;
        grid_map::Position footprint_point_2;
        drawSignFootprintFromSignOriginToNearestWalls(map, ORIGIN_LAYER, AUGMENTED_LAYER,
                                                      sign_center,
                                                      edge_intersection_1, edge_intersection_2,
                                                      footprint_point_1, footprint_point_2);
        response.x1_intersection = footprint_point_1.x();
        response.y1_intersection = footprint_point_1.y();
        response.x2_intersection = footprint_point_2.x();
        response.y2_intersection = footprint_point_2.y();

        nav_msgs::OccupancyGrid augmentedMapOccupancyGrid;
        grid_map::GridMapRosConverter::toOccupancyGrid(*map, AUGMENTED_LAYER,0.0, 1.0, augmentedMapOccupancyGrid);

        pub_map.publish(augmentedMapOccupancyGrid);
    }
    response.success = success_1 && success_2;
    return success_1 && success_2;
}

bool removeSignFromMap(simulation::RemoveSignFromMapMsg::Request &request, simulation::RemoveSignFromMapMsg::Response &response){
    grid_map::Position sign_center(request.x_center, request.y_center);
    grid_map::Position footprint_point_1(request.x1_intersection, request.y1_intersection);
    grid_map::Position footprint_point_2(request.x2_intersection, request.y2_intersection);

    bool success = eraseSignFootprint(map, AUGMENTED_LAYER, sign_center, footprint_point_1, footprint_point_2);

    response.success = success;
    if(success){
        nav_msgs::OccupancyGrid augmentedMapOG;
        grid_map::GridMapRosConverter::toOccupancyGrid(*map, AUGMENTED_LAYER,0.0, 1.0, augmentedMapOG);

        pub_map.publish(augmentedMapOG);
    }
    return true;
}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "map_enrichment_node");
    ros::NodeHandle nh("~");
    ros::ServiceClient client_map_server = nh.serviceClient<nav_msgs::GetMap>(REQ_ORIGINAL_MAP);
    ros::ServiceServer srv_add_sign = nh.advertiseService(ADV_ENRICH_AUGMENTED_MAP, addSignOnMap);
    ros::ServiceServer srv_remove_sign = nh.advertiseService(ADV_IMPOVERISH_AUGMENTED_MAP, removeSignFromMap);
    pub_map = nh.advertise<nav_msgs::OccupancyGrid>(PUB_AUGMENTED_MAP, 1, true);

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
