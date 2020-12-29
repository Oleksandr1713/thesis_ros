#ifndef SRC_DB_PROXY_DECORATOR_H
#define SRC_DB_PROXY_DECORATOR_H

#include "mongodb_store/message_store.h"
#include "simulation/DatabaseEntryInsert.h"
#include "simulation/DatabaseEntryUpdate.h"

using namespace mongodb_store;
using namespace std;
using namespace simulation;

namespace database_decorator {

    string insertNewEntry(MessageStoreProxy& db_proxy, DatabaseEntryInsert& entry){
        /* Populate database with a new entry and return its ID*/
        string id = db_proxy.insert(entry);
        entry.id = id;
        db_proxy.updateID(id, entry);
        return id;
    }

    boost::shared_ptr<DatabaseEntryInsert> getEntryById(MessageStoreProxy& db_proxy, const string& entry_id){
        vector<boost::shared_ptr<DatabaseEntryInsert>> results;
        boost::shared_ptr<DatabaseEntryInsert> entry;
        if(db_proxy.queryID<DatabaseEntryInsert>(entry_id, results)){
            entry = results[0];
        }
        results.clear();
        return entry;
    }

    bool updateEntry(MessageStoreProxy& db_proxy, const string& entry_id, DatabaseEntryUpdate& updateFields){
        boost::shared_ptr<DatabaseEntryInsert> dbEntry = getEntryById(db_proxy, entry_id);
        dbEntry->object_id = updateFields.object_id;
        dbEntry->job_id = updateFields.job_id;
        dbEntry->time_start = updateFields.time_start;
        dbEntry->time_end = updateFields.time_end;
        return db_proxy.updateID(entry_id, dbEntry.operator*());
    }

    bool deleteEntry(MessageStoreProxy& db_proxy, const string& entry_id){
        /* Delete database entry of a specified ID and return TRUE in case of success, otherwise FALSE*/
        return db_proxy.deleteID(entry_id);
    }

    boost::shared_ptr<DatabaseEntryInsert> getEntryByJobIdField(MessageStoreProxy& db_proxy, const int& job_id){
        vector<boost::shared_ptr<DatabaseEntryInsert>> results;
        boost::shared_ptr<DatabaseEntryInsert> entry;

        mongo::BSONObjBuilder message_query;
        message_query.append("job_id", job_id);

        if(db_proxy.query<DatabaseEntryInsert>(results, message_query.obj())){
            entry = results[0];
        }
        results.clear();
        return entry;
    }

    vector<boost::shared_ptr<DatabaseEntryInsert>> getAllEntries(MessageStoreProxy& db_proxy){
        vector<boost::shared_ptr<DatabaseEntryInsert>> results;
        db_proxy.query<DatabaseEntryInsert>(results);
        return results;
    }

    bool dropCollection(MessageStoreProxy& db_proxy){
        return db_proxy.dropCollection();
    }
}

#endif //SRC_DB_PROXY_DECORATOR_H
