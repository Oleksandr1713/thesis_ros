#include "ros/ros.h"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/GetMap.h"
#include "std_msgs/String.h"

//grid_map::GridMap *obstacles;
//grid_map::GridMap *map;
//nav_msgs::OccupancyGrid *occupancyGridInput;

ros::Publisher publisher_obstacles_found;


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
 * @return.
 */
void findIntersectionsOfLineWithHeightEdgesOfMapFrame(grid_map::Position& firstPoint, grid_map::Position& secondPoint,
                                                        const grid_map::GridMap* layers, const std::string layer_name,
                                                        grid_map::Position& coordLeftEdgeInter, grid_map::Position& coordRightEdgeInter){
    grid_map::Position bottom_left;
    grid_map::Position top_right;
    findLayerDimensionCoordinates(layers, layer_name, bottom_left, top_right);

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

/**
 * Method defines the coordinates of a sign footprint and displays this footprint on the layer as a line drawing.
 *
 * @param layers A data structure containing a list of layers of a navigation environment.
 * @param layer_name The name of desired layer, that is needed to work.
 * @param signLeftEdgeCoordinate The coordinate point of the left edge of the sign.
 * @param signRightEdgeCoordinate The coordinate point of the right edge of the sign.
 * @param footprintFirstCoordinate Ref to the first undefined coordinate, that describes a sign footprint.
 * @param footprintSecondCoordinate Ref to the second undefined coordinate, that describes a sign footprint.
 * @return.
 */
void drawSignFootprint(grid_map::GridMap* layers, const std::string layer_name,
                        grid_map::Position& signLeftEdgeCoordinate, grid_map::Position& signRightEdgeCoordinate,
                        grid_map::Position& footprintFirstCoordinate, grid_map::Position& footprintSecondCoordinate){

    grid_map::Position coordLeftEdgeInter;
    grid_map::Position coordRightEdgeInter;
    findIntersectionsOfLineWithHeightEdgesOfMapFrame(signLeftEdgeCoordinate, signRightEdgeCoordinate, layers, layer_name, coordLeftEdgeInter, coordRightEdgeInter);

    for (grid_map::LineIterator iterator(*layers, signLeftEdgeCoordinate, coordLeftEdgeInter); !iterator.isPastEnd(); ++iterator) {
        if(layers->at(layer_name, *iterator) > 0.8){
            layers->getPosition(*iterator, footprintFirstCoordinate);
            break;
        }
    }
    for (grid_map::LineIterator iterator(*layers, signLeftEdgeCoordinate, coordRightEdgeInter); !iterator.isPastEnd(); ++iterator) {
        if(layers->at(layer_name, *iterator) > 0.8){
            layers->getPosition(*iterator, footprintSecondCoordinate);
            break;
        }
    }
    for (grid_map::LineIterator iterator(*layers, footprintFirstCoordinate, footprintSecondCoordinate); !iterator.isPastEnd(); ++iterator) {
        layers->at(layer_name, *iterator) = 1.0;
    }
}


void drawPoint(grid_map::GridMap* layers, const std::string layer_name, grid_map::Position& point){
    for (grid_map::CircleIterator it(*layers, point, 0.05); !it.isPastEnd(); ++it) {
        layers->at(layer_name, *it) = 1;
    }
}

//void gridMapCallback(const nav_msgs::OccupancyGridConstPtr &msg) {
//    occupancyGridInput = new nav_msgs::OccupancyGrid();
//    occupancyGridInput->header = msg->header;
//    occupancyGridInput->info = msg->info;
//    occupancyGridInput->data = msg->data;
//
//    map = new grid_map::GridMap({"input_og"});
//    grid_map::GridMapRosConverter::fromOccupancyGrid(*occupancyGridInput, "input_og", *map);
//    map->setFrameId("map");
//    obstacles->addDataFrom(*map, true, true, true);
//
//    for (grid_map::GridMapIterator it(*obstacles); !it.isPastEnd(); ++it) {
//        if (obstacles->at("input_og", *it) > 0) {
//            obstacles->at("obstacles_found", *it) = 1;
//            if (obstacles->at("rgbd_scan", *it) > 0) {
//                obstacles->at("pending_obstacles", *it) = 0;
//            } else {
//                obstacles->at("pending_obstacles", *it) = 1;
//            }
//        } else {
//            obstacles->at("obstacles_found", *it) = 0;
//            obstacles->at("pending_obstacles", *it) = 0;
//        }
//    }
//
////    grid_map::GridMapRosConverter::toOccupancyGrid(*obstacles, "obstacles_found",
////                                                   0.0, 1.0, occupancyGridResult);
////    publisher_obstacles_found.publish(occupancyGridResult);
////    grid_map::GridMapRosConverter::toOccupancyGrid(*obstacles, "pending_obstacles",
////                                                   0.0, 1.0, occupancyGridResult);
////    publisher_pending_obstacles.publish(occupancyGridResult);
////    if (is_area_free(nearest_obstacle, 0.1)) {
////        set_new_goal();
////    }
////    if (!sm->check_obstacle_surrounding(
////            &current_robot_position, &obstacle_bearing, camera_view_dist,
////            min_dist, current_obstacle, obstacles, map)) {
////        set_new_goal();
////    }
//}

void callbackMapModifier(const std_msgs::String& msg){
    ROS_INFO("I received: [%s]", msg.entry_id.c_str());

}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "map_enrichment_node");
    ros::NodeHandle nh("~");
    ros::ServiceClient client_map_server = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    ros::Publisher pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/augmented_map", 1, true);
    ros::Publisher pub_signs = nh.advertise<nav_msgs::OccupancyGrid>("/signs", 1);
    ros::Subscriber sub_map_modifier = nh.subscribe("/map_modification", 1, callbackMapModifier);

    nav_msgs::GetMap srv;
    client_map_server.call(srv);
    nav_msgs::OccupancyGrid *pStaticMapOccupancyGrid = &srv.response.map;
    /*ROS_INFO("Static map size: %i", pStaticMapOccupancyGrid->data.size());
    ROS_INFO("width: %i", pStaticMapOccupancyGrid->info.width);
    ROS_INFO("height: %i", pStaticMapOccupancyGrid->info.height);*/
    pub_map.publish(pStaticMapOccupancyGrid);

    grid_map::GridMap *map = new grid_map::GridMap({"static_map_layer", "signs_layer"});
    grid_map::GridMapRosConverter::fromOccupancyGrid(*pStaticMapOccupancyGrid, "static_map_layer", *map);
    map->setFrameId(pStaticMapOccupancyGrid->header.frame_id);

    ROS_INFO("Map grid with size %.2f x %.2f m (%i x %i cells).",
             map->getLength().x(), map->getLength().y(), map->getSize()(0), map->getSize()(1));
    ROS_INFO("Map grid with (x=%f, y=%f) center coordinates.", map->getPosition()(0), map->getPosition()(1));


//    grid_map::GridMap *obstacles = new grid_map::GridMap({"sign_layer"});
//    obstacles->setFrameId(map->getFrameId());
//    obstacles->setGeometry(map->getLength(), map->getResolution());
//    obstacles->setPosition(map->getPosition());

//    grid_map::Position point(-1.75,1.13);
//
//    for (grid_map::CircleIterator it(*map, point, 0.05); !it.isPastEnd(); ++it) {
//        map->at("static_map_layer", *it) = 1;
//    }


    // the size of a navigation map relative to the ros MAP frame coordinate system
    // (the bottom left point and the top right point of the map)
    grid_map::Position bottom_left;
    grid_map::Position top_right;
    findLayerDimensionCoordinates(map, "signs_layer", bottom_left, top_right);
    ROS_INFO("Bottom-Left coordinates: (x=%f, y=%f)", bottom_left.x(), bottom_left.y());
    ROS_INFO("Top-Right coordinates: (x=%f, y=%f)", top_right.x(), top_right.y());


    grid_map::Position point_1(-2,1);
    grid_map::Position point_2(-3,0.5);
//    drawPoint(map, "static_map_layer", point_1);
//    drawPoint(map, "static_map_layer", point_2);


//    float slope = (point_1.y() - point_2.y())/(point_1.x() - point_2.x());
//    float x;
//    float y = slope * x - (slope * point_1.x() - point_1.y());

    grid_map::Position coordLeftInter;
    grid_map::Position coordRightInter;
    findIntersectionsOfLineWithHeightEdgesOfMapFrame(point_1, point_2, map, "static_map_layer", coordLeftInter, coordRightInter);
    ROS_INFO("Left intersection coordinates: (x=%f, y=%f)", coordLeftInter.x(), coordLeftInter.y());
    ROS_INFO("Right intersection coordinates: (x=%f, y=%f)", coordRightInter.x(), coordRightInter.y());
    drawPoint(map, "static_map_layer", coordLeftInter);
    drawPoint(map, "static_map_layer", coordRightInter);


    grid_map::Position firstConnectPoint;
    grid_map::Position secondConnectPoint;
    drawSignFootprint(map, "static_map_layer", point_1, point_2, firstConnectPoint, secondConnectPoint);

    nav_msgs::OccupancyGrid occupancyGridResult;
    grid_map::GridMapRosConverter::toOccupancyGrid(*map, "static_map_layer",0.0, 1.0, occupancyGridResult);


//    publisher_signs.publish(occupancyGridResult);



//    map->setGeometry(grid_map::Length(600, 600),
//                    0.01);
//    grid_map::GridMap obstacles;
//    grid_map::Length(occupancyGridInput->info.width, occupancyGridInput->info.height),


//    beginner_tutorials::AddTwoInts srv;
//    srv.request.a = atoll(argv[1]);
//    srv.request.b = atoll(argv[2]);
//    if (client.call(srv))
//    {
//        ROS_INFO("Sum: %ld", (long int)srv.response.sum);
//    }
//    else
//    {
//        ROS_ERROR("Failed to call service add_two_ints");
//        return 1;
//    }
//
//    ros::Subscriber sub = nh.subscribe("/catcher", 1, callback);

    ROS_INFO("Map Enrichment node is up.");
    ros::Rate rate(10);
    while (nh.ok()){
        pub_signs.publish(occupancyGridResult);
        ros::spinOnce();
        rate.sleep();
    }
//    ros::spin();
    return 0;
}
