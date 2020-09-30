#include "ros/ros.h"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/GetMap.h"
#include "simulation/AddSignOnMapMsg.h"
#include "simulation/RemoveSignFromMapMsg.h"
#include "my_lib/auxiliary_func.h"
#include "constants/node_constants.h"

#include "simulation/DatabaseEntryInsert.h"
#include "mongodb_store/message_store.h"
#include "database/db_proxy_decorator.h"

using namespace auxiliary_func;
using namespace simulation;
using namespace mongodb_proxy_decorator;

/* ROS node name*/
constexpr static const char* NODE_NAME = "map_enrichment_node";

class MapEnrichment{

public:
    /* Layer names */
    const std::string ORIGIN_LAYER = "origin_map_layer";
    const std::string AUGMENTED_LAYER = "augmented_map_layer";

    /* Node's constants */
    const double UNIT_RADIUS = 0.05;    // meters
    const double AZIMUTH = 95;          // degrees

private:
    ros::NodeHandle nh_base;
    grid_map::GridMap *map;

    ros::ServiceClient client_map_server;
    ros::ServiceServer srv_add_sign;
    ros::ServiceServer srv_remove_sign;

    ros::Publisher pub_map;
    ros::Publisher pub_signs;
    ros::Publisher pub_map_update;


public:
    explicit MapEnrichment(): nh_base(){
        client_map_server = nh_base.serviceClient<nav_msgs::GetMap>(str(node_constants::REQ_ORIGINAL_MAP));
        srv_add_sign = nh_base.advertiseService(str(node_constants::ADV_ENRICH_AUGMENTED_MAP), &MapEnrichment::addSignOnMap, this);
        srv_remove_sign = nh_base.advertiseService(str(node_constants::ADV_IMPOVERISH_AUGMENTED_MAP), &MapEnrichment::removeSignFromMap, this);
        pub_map = nh_base.advertise<nav_msgs::OccupancyGrid>(str(node_constants::PUB_AUGMENTED_MAP), 1, true);

        nav_msgs::GetMap srv;
        client_map_server.call(srv);
        nav_msgs::OccupancyGrid *pStaticMapOccupancyGrid = &srv.response.map;
        pub_map.publish(*pStaticMapOccupancyGrid);

        map = new grid_map::GridMap({ORIGIN_LAYER, AUGMENTED_LAYER});
        grid_map::GridMapRosConverter::fromOccupancyGrid(*pStaticMapOccupancyGrid, ORIGIN_LAYER, *map);
        grid_map::GridMapRosConverter::fromOccupancyGrid(*pStaticMapOccupancyGrid, std::string(AUGMENTED_LAYER), *map);
        map->setFrameId(pStaticMapOccupancyGrid->header.frame_id);

        ROS_INFO("Map grid with size %.2f x %.2f m (%i x %i cells).",
                 map->getLength().x(), map->getLength().y(), map->getSize()(0), map->getSize()(1));
        ROS_INFO("Map grid with (x=%f, y=%f) center coordinates.", map->getPosition()(0), map->getPosition()(1));
        ROS_INFO("Map Enrichment node is up.");
    }

    ~MapEnrichment(){
        ROS_INFO("Map Enrichment node is destroyed.");
    }

private:
    bool isAzimuthValid(double azimuth){
        return (0 < std::abs (azimuth) && std::abs (azimuth) < 180);
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

    std::string saveDataToDatabase(simulation::AddSignOnMapMsg::Request &request, simulation::AddSignOnMapMsg::Response &response){
        MessageStoreProxy messageStore(nh_base, str(node_constants::COLLECTION_NAME));
        DatabaseEntryInsert dbEntry;
        dbEntry.x_center = request.x_center;
        dbEntry.y_center = request.y_center;
        dbEntry.yaw = request.dir_radians;
        dbEntry.x1_intersection = response.x1_intersection;
        dbEntry.y1_intersection = response.y1_intersection;
        dbEntry.x2_intersection = response.x2_intersection;
        dbEntry.y2_intersection = response.y2_intersection;
        return insertNewEntry(messageStore, dbEntry);
    }

    bool deleteDataFromDatabase(string const& entry_id){
        MessageStoreProxy messageStore(nh_base, str(node_constants::COLLECTION_NAME));
        return deleteEntry(messageStore, entry_id);
    }

    boost::shared_ptr<DatabaseEntryInsert> getDataByIdFromDatabase(string const& entry_id){
        MessageStoreProxy messageStore(nh_base, str(node_constants::COLLECTION_NAME));
        return getEntryById(messageStore, entry_id);
    }

    bool addSignOnMap(simulation::AddSignOnMapMsg::Request &request, simulation::AddSignOnMapMsg::Response &response){
        grid_map::Position sign_center(request.x_center, request.y_center);
        double sign_direction = getDegrees(request.dir_radians); // degrees
        ROS_DEBUG("ME_1");
        grid_map::Position edge_intersection_1;
        grid_map::Position edge_intersection_2;
        bool success_1 = findIntersectionPointWithLayerEdge(map, AUGMENTED_LAYER, sign_center, sign_direction, AZIMUTH, edge_intersection_1);
        bool success_2 = findIntersectionPointWithLayerEdge(map, AUGMENTED_LAYER, sign_center, sign_direction, -AZIMUTH, edge_intersection_2);
        ROS_DEBUG("ME_2");
        ROS_DEBUG_STREAM("" << success_1 << " " << success_2);
        if(success_1 && success_2) {
            grid_map::Position footprint_point_1;
            grid_map::Position footprint_point_2;
            ROS_DEBUG("ME_3");
            drawSignFootprintFromSignOriginToNearestWalls(map, ORIGIN_LAYER, AUGMENTED_LAYER,
                                                          sign_center,
                                                          edge_intersection_1, edge_intersection_2,
                                                          footprint_point_1, footprint_point_2);
            ROS_DEBUG("ME_4");
            response.x1_intersection = footprint_point_1.x();
            response.y1_intersection = footprint_point_1.y();
            response.x2_intersection = footprint_point_2.x();
            response.y2_intersection = footprint_point_2.y();

            std::string entry_id = saveDataToDatabase(request,response);
            ROS_DEBUG("ME_5");

            response.entry_id = entry_id;

            nav_msgs::OccupancyGrid augmentedMapOccupancyGrid;
            grid_map::GridMapRosConverter::toOccupancyGrid(*map, AUGMENTED_LAYER,0.0, 1.0, augmentedMapOccupancyGrid);

            pub_map.publish(augmentedMapOccupancyGrid);
            ROS_DEBUG("ME_6");

        }
        response.success = success_1 && success_2;
        return success_1 && success_2;
    }

    bool removeSignFromMap(simulation::RemoveSignFromMapMsg::Request &request, simulation::RemoveSignFromMapMsg::Response &response){
        boost::shared_ptr<DatabaseEntryInsert> dbEntry = getDataByIdFromDatabase(request.entry_id);

        grid_map::Position sign_center(dbEntry->x_center, dbEntry->y_center);
        grid_map::Position footprint_point_1(dbEntry->x1_intersection, dbEntry->y1_intersection);
        grid_map::Position footprint_point_2(dbEntry->x2_intersection, dbEntry->y2_intersection);

        bool success = eraseSignFootprint(map, AUGMENTED_LAYER, sign_center, footprint_point_1, footprint_point_2);

        response.success = success;
        if(success){
            response.x_center = dbEntry->x_center;
            response.y_center = dbEntry->y_center;

            deleteDataFromDatabase(request.entry_id);

            nav_msgs::OccupancyGrid augmentedMapOG;
            grid_map::GridMapRosConverter::toOccupancyGrid(*map, AUGMENTED_LAYER,0.0, 1.0, augmentedMapOG);

            pub_map.publish(augmentedMapOG);
        }
        return success;
    }
};


int main(int argc, char * argv[]) {
    /*changeNodeLoggerLevel(ros::console::levels::Debug);*/
    ros::init(argc, argv, std::string(NODE_NAME));
    MapEnrichment mapEnrichment;
    ros::spin();
    return 0;
}
