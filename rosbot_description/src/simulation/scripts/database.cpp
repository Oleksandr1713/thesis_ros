#include "ros/ros.h"
#include "mongodb_store/message_store.h"

#include "simulation/DatabaseEntry.h"


using namespace mongodb_store;
using namespace std;
using namespace simulation;


string insertNewEntry(MessageStoreProxy& db_proxy, DatabaseEntry& entry){
    /* Populate database with a new entry and return its ID*/
    string id = db_proxy.insert(entry);
    entry.id = id;
    db_proxy.updateID(id, entry);
    return id;
}

bool updateEntry(MessageStoreProxy& db_proxy, const string& entry_id, DatabaseEntry& entry){
    return db_proxy.updateID(entry_id, entry);
}

bool deleteEntry(MessageStoreProxy& db_proxy, const string& entry_id){
    /* Delete database entry of a specified ID and return TRUE in case of success, otherwise FALSE*/
    return db_proxy.deleteID(entry_id);
}

boost::shared_ptr<DatabaseEntry> getEntryById(MessageStoreProxy& db_proxy, const string& entry_id){
    vector<boost::shared_ptr<DatabaseEntry>> results;
    boost::shared_ptr<DatabaseEntry> entry;
    if(db_proxy.queryID<DatabaseEntry>(entry_id, results)){
        entry = results[0];
    }
    results.clear();
    return entry;
}

boost::shared_ptr<DatabaseEntry> getEntryByJobIdField(MessageStoreProxy& db_proxy, const int& job_id){
    vector<boost::shared_ptr<DatabaseEntry>> results;
    boost::shared_ptr<DatabaseEntry> entry;

    mongo::BSONObjBuilder message_query;
    message_query.append("job_id", job_id);

    if(db_proxy.query<DatabaseEntry>(results, message_query.obj())){
        entry = results[0];
    }
    results.clear();
    return entry;
}

vector<boost::shared_ptr<DatabaseEntry>> getAllEntries(MessageStoreProxy& db_proxy){
    vector<boost::shared_ptr<DatabaseEntry>> results;
    db_proxy.query<DatabaseEntry>(results);
    return results;
}

//bool dropCollection(MessageStoreProxy& db_proxy){
//    return db_proxy.dropCollection();
//}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "example");
    ros::NodeHandle nh;

    string collectionName = "ros_sign_info";
    MessageStoreProxy messageStore(nh, collectionName);
    DatabaseEntry dbEntry;
    dbEntry.x_coordinate = 10;
    dbEntry.job_id = 777;

    // insert entry
    string id = insertNewEntry(messageStore, dbEntry);
    ROS_INFO_STREAM("create ID: " << id);

//    // update entry
//    dbEntry.x_coordinate = 111;
//    dbEntry.job_id = 111;
//    updateEntry(messageStore, id, dbEntry);

//    // remove entry
//    bool result = deleteEntry(messageStore, id);
//    if(result){
//        ROS_INFO_STREAM("delete ID: " << id);
//    } else{
//        ROS_INFO_STREAM("delete problem");
//    }

//    // get entry by id
//    boost::shared_ptr<DatabaseEntry> entry = getEntryById(messageStore, id);
//    ROS_INFO_STREAM("job_id of returned entry: " << entry->job_id);

//    // get entry by job_id field
//    boost::shared_ptr<DatabaseEntry> entry = getEntryByJobIdField(messageStore, dbEntry.job_id);
//    ROS_INFO_STREAM("id of returned entry: " << entry->id);

    // get all documents from the collection
    ROS_INFO_STREAM("Num of entries: " << getAllEntries(messageStore).size());

//    // drop collection and its data from database
//    messageStore.dropCollection();



    ros::spinOnce();
    return 0;
}


//    // insert entry
//    string id = insertNewEntry(messageStore, dbEntry);
//    ROS_INFO_STREAM("create ID: " << id);

//    // update entry
//    dbEntry.x_coordinate = 111;
//    dbEntry.job_id = 111;
//    updateEntry(messageStore, "5ee94d6f9f78ce127d25569c", dbEntry);

//    // remove entry
//    bool result = messageStore.deleteID(id);
//    if(result){
//        ROS_INFO_STREAM("delete ID: " << id);
//    } else{
//        ROS_INFO_STREAM("delete problem");
//    }

//    // get entry by id
//    id = messageStore.insert(dbEntry);
//
//    vector<boost::shared_ptr<DatabaseEntry> > results;
//    if(messageStore.queryID<DatabaseEntry>(id, results)){
////        results[0];
//    }
//    results.clear();

//    // get entry by job_id field
//    vector<boost::shared_ptr<DatabaseEntry> > results;
//    mongo::BSONObjBuilder message_query;
//    message_query.append("job_id", 10);
//    if(messageStore.query<DatabaseEntry>(results, message_query.obj())){
//        ROS_INFO_STREAM(results.size());
//    }
//    results.clear();

// drop collection and its data from database
//    messageStore.dropCollection();

