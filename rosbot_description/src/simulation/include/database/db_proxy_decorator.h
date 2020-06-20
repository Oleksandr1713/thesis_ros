#ifndef SRC_DB_PROXY_DECORATOR_H
#define SRC_DB_PROXY_DECORATOR_H

#include "mongodb_store/message_store.h"
#include "simulation/DatabaseEntry.h"

using namespace mongodb_store;
using namespace std;
using namespace simulation;

namespace mongodb_proxy_decorator {

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

    bool dropCollection(MessageStoreProxy& db_proxy){
        return db_proxy.dropCollection();
    }
}

#endif //SRC_DB_PROXY_DECORATOR_H
