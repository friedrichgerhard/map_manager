/* *****************************************************************
 *
 * map_manager
 *
 * Copyright (c) 2019
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/

/**
 * @file   dbdriver.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Driver for accessing the database linkpoint.db
 */

#include "dbdriver/dbdriver.h"

sqlite3* DBDriver::open_database(std::string url)
{
    // ##### open database #####
    sqlite3 *conn =0; //Database struct handle
    int dbhandle; //Resource handle
    dbhandle = sqlite3_open_v2(url.c_str(), &conn, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 0);

    if(dbhandle){
        //error
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(conn));
        sqlite3_close(conn);
        ROS_ERROR("Database opening failed!");
    }
    return conn;
}

//################### FUNCTIONS FOR TABLE LINKPOINTS ########################################################
void DBDriver::create_table_linkpoints(std::string url)
{
   sqlite3 *conn = DBDriver::open_database(url);
   char *sql;
   char *zErrMsg = 0;
   int rc;
    /* Create SQL statement */
   sql = "CREATE TABLE LINKPOINTS("  \
      "ID               INT   PRIMARY KEY     NOT NULL," \
      "TAG              TEXT                          ," \
      "MAP_ID_0         INT                   NOT NULL," \
      "POS_X_0          REAL                  NOT NULL," \
      "POS_Y_0          REAL                  NOT NULL," \
      "THETA_0          REAL                  NOT NULL," \
      "MAP_ID_1         INT                           ," \
      "POS_X_1          REAL                          ," \
      "POS_Y_1          REAL                          ," \
      "THETA_1          REAL                          ," \
      "POS_X_WORLD      REAL                          ," \
      "POS_Y_WORLD      REAL                          ," \
      "THETA_WORLD      REAL                          ," \
      "IMAGE_FRONT      BLOB                          ," \
      "IMAGE_REAR       BLOB                          ," \
      "DIRECTION        INT                          ," \
      "DIFFICULTY       INT                           ," \
      "ENVIRONMENT      INT                           );";

   /* Execute SQL statement */
   rc = sqlite3_exec(conn, sql, NULL, 0, &zErrMsg);

   if( rc != SQLITE_OK ){
      fprintf(stderr, "SQL error: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
   } else {
      fprintf(stdout, "Table linkpoints created successfully\n");
   }
   sqlite3_close(conn);
}


bool DBDriver::insert_linkpoint(std::string url, LinkPoint lp)
{
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;

    /* Create SQL statement */
    std::stringstream sql_ss;

    sql_ss << "INSERT INTO LINKPOINTS (ID,TAG,MAP_ID_0,POS_X_0,POS_Y_0,THETA_0,MAP_ID_1,POS_X_1,POS_Y_1,THETA_1,POS_X_WORLD,POS_Y_WORLD,THETA_WORLD,IMAGE_FRONT,IMAGE_REAR,DIRECTION,DIFFICULTY,ENVIRONMENT) "
              << "VALUES ("
              << lp.get_id() << ","
              << "'" << lp.get_tag() << "'" << ","
              << lp.get_map_id(0) << ","
              << lp.get_pos_x(0) << ","
              << lp.get_pos_y(0) << ","
              << lp.get_theta(0) << ","
              << lp.get_map_id(1) << ","
              << lp.get_pos_x(1) << ","
              << lp.get_pos_y(1) << ","
              << lp.get_theta(1) << ","
              << lp.get_pos_x(2) << ","
              << lp.get_pos_y(2) << ","
              << lp.get_theta(2) << ","
              << "NULL" << ","
              << "NULL" << ","
              << lp.get_direction() << ","
              << lp.get_difficulty() << ","
              << lp.get_environment()
              <<");";

    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    /* Execute SQL statement */
       rc = sqlite3_exec(conn, sql, NULL, 0, &zErrMsg);

       bool success;
       if( rc != SQLITE_OK ){
          fprintf(stderr, "SQL error: %s\n", zErrMsg);
          sqlite3_free(zErrMsg);
          success = false;
       } else {
          fprintf(stdout, "Insertion was successful\n");
          success = true;
       }
       sqlite3_close(conn);
       return success;
}


bool DBDriver::insert_linkpoint_with_images(std::string url, LinkPoint lp, cv::Mat img_front, cv::Mat img_rear)
{
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;
    sqlite3_stmt* stmtInsert = 0;

    /* Create SQL statement */
    std::stringstream sql_ss;

    sql_ss << "INSERT INTO LINKPOINTS (ID,TAG,MAP_ID_0,POS_X_0,POS_Y_0,THETA_0,MAP_ID_1,POS_X_1,POS_Y_1,THETA_1,POS_X_WORLD,POS_Y_WORLD,THETA_WORLD,IMAGE_FRONT,IMAGE_REAR,DIRECTION,DIFFICULTY,ENVIRONMENT) "
              << "VALUES ("
              << lp.get_id() << ","
              << "'" << lp.get_tag() << "'" << ","
              << lp.get_map_id(0) << ","
              << lp.get_pos_x(0) << ","
              << lp.get_pos_y(0) << ","
              << lp.get_theta(0) << ","
              << lp.get_map_id(1) << ","
              << lp.get_pos_x(1) << ","
              << lp.get_pos_y(1) << ","
              << lp.get_theta(1) << ","
              << lp.get_pos_x(2) << ","
              << lp.get_pos_y(2) << ","
              << lp.get_theta(2) << ","
              << "?, "
              << "?, "
              << lp.get_direction() << ","
              << lp.get_difficulty() << ","
              << lp.get_environment()
              <<");";

    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    //prepare sql statement
    sqlite3_prepare_v2(conn, sql, -1, &stmtInsert, 0);

    //bind image front
    cv::Mat imageCompressed_front = DBDriver::compress_image(img_front);
    std::cout << "Compressed Image Front:\n";
    std::cout << "Cols:" << imageCompressed_front.cols << "\n";
    std::cout << "Rows:" << imageCompressed_front.rows << "\n";
    std::cout << "data:" << imageCompressed_front.data << "\n\n";
    rc = sqlite3_bind_blob(stmtInsert, 1, imageCompressed_front.data, imageCompressed_front.cols, SQLITE_STATIC);

    //bind image rear
    cv::Mat imageCompressed_rear = DBDriver::compress_image(img_rear);
    std::cout << "Compressed Image Front:\n";
    std::cout << "Cols:" << imageCompressed_rear.cols << "\n";
    std::cout << "Rows:" << imageCompressed_rear.rows << "\n";
    std::cout << "data:" << imageCompressed_rear.data << "\n\n";
    rc = sqlite3_bind_blob(stmtInsert, 2, imageCompressed_rear.data, imageCompressed_rear.cols, SQLITE_STATIC);

    //execute statement
    rc = sqlite3_step(stmtInsert);
    if (rc != SQLITE_DONE){
        std::cerr << "execution failed!" << sqlite3_errmsg(conn) << std::endl;
        sqlite3_finalize(stmtInsert);
        sqlite3_close(conn);
        return false;
     }
    sqlite3_finalize(stmtInsert);
    sqlite3_close(conn);
    return true;
}

int DBDriver::get_db_property_int(std::string url, int id, std::string property, std::string table)
{
    sqlite3 *conn = DBDriver::open_database(url);

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT " << property.c_str() << " FROM " << table.c_str() << " WHERE ID = " << id <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    if(sqlite3_step(stmtRetrieve) == SQLITE_ROW)
        {
        int value;
        value = sqlite3_column_int(stmtRetrieve, 0);
//        ROS_INFO("Value = %i", value);
        sqlite3_finalize(stmtRetrieve);
        sqlite3_close(conn);
        return value;
        }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    ROS_ERROR("Error retrieving property %s for linkpoint id %i", property.c_str(), id);
}


double DBDriver::get_db_property_double(std::string url, int id, std::string property, std::string table)
{
    sqlite3 *conn = DBDriver::open_database(url);

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT " << property.c_str() << " FROM " << table.c_str() << " WHERE ID = " << id <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    if(sqlite3_step(stmtRetrieve) == SQLITE_ROW)
        {
        double value;
        value = sqlite3_column_double(stmtRetrieve, 0);
//        ROS_INFO("Value = %f", value);
        sqlite3_finalize(stmtRetrieve);
        sqlite3_close(conn);
        return value;
        }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    ROS_ERROR("Error retrieving property %s for linkpoint id %i", property.c_str(), id);
}

std::string DBDriver::get_db_property_text(std::string url, int id, std::string property, std::string table)
{
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;
    const char* data = "Callback function called";

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT " << property.c_str() << " FROM " << table.c_str() << " WHERE ID = " << id <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    if(sqlite3_step(stmtRetrieve) == SQLITE_ROW)
        {
            const unsigned char *first;
            first = sqlite3_column_text(stmtRetrieve, 0);
            std::size_t s = sqlite3_column_bytes(stmtRetrieve, 0);
            //std::cout << std::string((const char*)first, s) << "\n"; //print entry
            sqlite3_finalize(stmtRetrieve);
            sqlite3_close(conn);
            return std::string((const char*)first, s);
        }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    ROS_ERROR("Error retrieving property %s for linkpoint id %i", property.c_str(), id);
}

LinkPoint DBDriver::get_linkpoint(std::string url, int id)
{
    LinkPoint lp;
    lp.set_id(id);
    lp.set_tag(DBDriver::get_db_property_text(url, id, "TAG", "LINKPOINTS"));
    lp.set_map_id(DBDriver::get_db_property_int(url, id, "MAP_ID_0", "LINKPOINTS"), 0);
    lp.set_pos_x(DBDriver::get_db_property_double(url, id, "POS_X_0", "LINKPOINTS"), 0);
    lp.set_pos_y(DBDriver::get_db_property_double(url, id, "POS_Y_0", "LINKPOINTS"), 0);
    lp.set_theta(DBDriver::get_db_property_double(url, id, "THETA_0", "LINKPOINTS"), 0);
    lp.set_map_id(DBDriver::get_db_property_int(url, id, "MAP_ID_1", "LINKPOINTS"), 1);
    lp.set_pos_x(DBDriver::get_db_property_double(url, id, "POS_X_1", "LINKPOINTS"), 1);
    lp.set_pos_y(DBDriver::get_db_property_double(url, id, "POS_Y_1", "LINKPOINTS"), 1);
    lp.set_theta(DBDriver::get_db_property_double(url, id, "THETA_1", "LINKPOINTS"), 1);
    lp.set_pos_x(DBDriver::get_db_property_double(url, id, "POS_X_WORLD", "LINKPOINTS"), 2);
    lp.set_pos_y(DBDriver::get_db_property_double(url, id, "POS_Y_WORLD", "LINKPOINTS"), 2);
    lp.set_theta(DBDriver::get_db_property_double(url, id, "THETA_WORLD", "LINKPOINTS"), 2);
    lp.set_direction(DBDriver::get_db_property_int(url, id, "DIRECTION", "LINKPOINTS"));
    lp.set_difficulty(DBDriver::get_db_property_int(url, id, "DIFFICULTY", "LINKPOINTS"));
    lp.set_environment(DBDriver::get_db_property_int(url, id, "ENVIRONMENT", "LINKPOINTS"));
    return lp;
}

int DBDriver::get_lp_id_by_tag(std::string url, std::string tag)
{
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;
    const char* data = "Callback function called";

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT ID FROM LINKPOINTS WHERE TAG = '" << tag <<"';";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    if(sqlite3_step(stmtRetrieve) == SQLITE_ROW)
        {
        int value;
        value = sqlite3_column_int(stmtRetrieve, 0);
        ROS_INFO("Value = %i", value);
        sqlite3_finalize(stmtRetrieve);
        sqlite3_close(conn);
        return value;
        }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    ROS_ERROR("Error retrieving linkpoint for tag '%s'.", tag);
}

cv::Mat DBDriver::get_linkpoint_image(std::string url, int id, std::string cam_position){
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;
    const char* data = "Callback function called";
    cv::Mat image;

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT " << cam_position.c_str() << " FROM LINKPOINTS WHERE ID = " << id <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    if(sqlite3_step(stmtRetrieve) == SQLITE_ROW){
        const void * data = 0;
        int dataSize = 0;

        data = sqlite3_column_blob(stmtRetrieve, 0);
        dataSize = sqlite3_column_bytes(stmtRetrieve, 0);

        if(dataSize>0 && data){
            image = rtabmap::uncompressImage(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
        }
        else{
            ROS_ERROR("Error retrieving image from database!");
        }
    }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    return image;
}

void DBDriver::delete_linkpoint(std::string url, int id){
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;

     /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "DELETE FROM LINKPOINTS WHERE ID = " << id <<";";

    std::string  str(sql_ss.str());
    const char *sql = str.c_str();
    /* Execute SQL statement */
    rc = sqlite3_exec(conn, sql, NULL, 0, &zErrMsg);

    if( rc != SQLITE_OK ){
       fprintf(stderr, "SQL error: %s\n", zErrMsg);
       sqlite3_free(zErrMsg);
    } else {
       fprintf(stdout, "Deleted linkpoint with id %i!!!\n", id);
    }
    sqlite3_close(conn);
}

std::vector<int> DBDriver::get_linkpoint_ids(std::string url){
    sqlite3 *conn = DBDriver::open_database(url);
    int rc;

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT " << "ID" << " FROM " << "LINKPOINTS" <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    std::vector<int> linkpoint_ids;
    rc = sqlite3_step(stmtRetrieve);
    while(rc == SQLITE_ROW)
    {
        linkpoint_ids.push_back(sqlite3_column_int(stmtRetrieve, 0));
        rc = sqlite3_step(stmtRetrieve);
    }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    return linkpoint_ids;
}

//################### FUNTIONS FOR TABLE MAPS ########################################################

void DBDriver::create_table_maps(std::string url)
{
   sqlite3 *conn = DBDriver::open_database(url);
   char *sql;
   char *zErrMsg = 0;
   int rc;

    /* Create SQL statement */
   sql = "CREATE TABLE MAPS("  \
      "MAP_ID         INT     PRIMARY KEY     NOT NULL," \
      "NAME           TEXT    NOT NULL," \
      "PATH           TEXT    NOT NULL," \
      "STRANDS_DB     TEXT            ," \
      "INDOOR         INT     NOT NULL);";

   /* Execute SQL statement */
   rc = sqlite3_exec(conn, sql, NULL, 0, &zErrMsg);

   if( rc != SQLITE_OK ){
      fprintf(stderr, "SQL error: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
   } else {
      fprintf(stdout, "Table maps created successfully\n");
   }
   sqlite3_close(conn);
}

void DBDriver::insert_map(std::string url, int map_id, std::string name, std::string path, bool indoor)
{
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;

    int indoor_int;
    if(indoor){indoor_int = 1;}
    else {indoor_int = 0;}

    /* Create SQL statement */
    std::stringstream sql_ss;

    sql_ss << "INSERT INTO MAPS (MAP_ID,NAME,PATH,STRANDS_DB,INDOOR) "
              << "VALUES ("
              << map_id << ","
              << "'" << name.c_str() << "',"
              << "'" << path.c_str() << "',"
              << "NULL,"
              << indoor_int
              << ");";

    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    /* Execute SQL statement */
       rc = sqlite3_exec(conn, sql, NULL, 0, &zErrMsg);

       if( rc != SQLITE_OK ){
          fprintf(stderr, "SQL error: %s\n", zErrMsg);
          sqlite3_free(zErrMsg);
       } else {
          fprintf(stdout, "Insertion was successful\n");
       }
       sqlite3_close(conn);
}

void DBDriver::insert_strandsDB_into_map(std::string url, int map_id, std::string strands_db){
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;

    /* Create SQL statement */
    std::stringstream sql_ss;

    std::string name = DBDriver::get_map_property_text(url, map_id, "NAME");
    std::string path = DBDriver::get_map_property_text(url, map_id, "PATH");
    int indoor_int = DBDriver::get_map_property_int(url, map_id, "INDOOR");

    DBDriver::delete_map(url, map_id);

    sql_ss << "INSERT INTO MAPS (MAP_ID,NAME,PATH,STRANDS_DB,INDOOR) "
              << "VALUES ("
              << map_id << ","
              << "'" << name.c_str() << "',"
              << "'" << path.c_str() << "',"
              << "'" << strands_db.c_str() << "',"
              << indoor_int
              << ");";

    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    /* Execute SQL statement */
       rc = sqlite3_exec(conn, sql, NULL, 0, &zErrMsg);

       if( rc != SQLITE_OK ){
          fprintf(stderr, "SQL error: %s\n", zErrMsg);
          sqlite3_free(zErrMsg);
       } else {
          fprintf(stdout, "Insertion was successful\n");
       }
       sqlite3_close(conn);
}

std::string DBDriver::get_map_property_text(std::string url, int id, std::string property)
{
    sqlite3 *conn = DBDriver::open_database(url);

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT " << property.c_str() << " FROM MAPS WHERE MAP_ID = " << id <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    if(sqlite3_step(stmtRetrieve) == SQLITE_ROW)
        {
            const unsigned char *first;
            first = sqlite3_column_text(stmtRetrieve, 0);
            std::size_t s = sqlite3_column_bytes(stmtRetrieve, 0);
            std::string entry = std::string((const char*)first, s);
            std::cout << "Entry inside funtion: " << entry << "\n"; //print entry
            sqlite3_finalize(stmtRetrieve);
            sqlite3_close(conn);
            return entry;
        }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    ROS_ERROR("Error retrieving property %s for map_id %i", property.c_str(), id);
}

int DBDriver::get_map_property_int(std::string url, int id, std::string property)
{
    sqlite3 *conn = DBDriver::open_database(url);

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT " << property.c_str() << " FROM MAPS WHERE MAP_ID = " << id <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    if(sqlite3_step(stmtRetrieve) == SQLITE_ROW)
        {
        int value;
        value = sqlite3_column_int(stmtRetrieve, 0);
        ROS_INFO("Value = %i", value);
        sqlite3_finalize(stmtRetrieve);
        sqlite3_close(conn);
        return value;
        }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    ROS_ERROR("Error retrieving property %s for map_id %i", property.c_str(), id);
}

void DBDriver::delete_map(std::string url, int map_id){
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;

     /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "DELETE FROM MAPS WHERE MAP_ID = " << map_id <<";";

    std::string  str(sql_ss.str());
    const char *sql = str.c_str();
    /* Execute SQL statement */
    rc = sqlite3_exec(conn, sql, NULL, 0, &zErrMsg);

    if( rc != SQLITE_OK ){
       fprintf(stderr, "SQL error: %s\n", zErrMsg);
       sqlite3_free(zErrMsg);
    } else {
       fprintf(stdout, "Deleted map with id %i!!!\n", map_id);
    }
    sqlite3_close(conn);
}

std::vector<int> DBDriver::get_map_ids(std::string url){
    sqlite3 *conn = DBDriver::open_database(url);
    int rc;

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT " << "MAP_ID" << " FROM " << "MAPS" <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    std::vector<int> map_ids;
    rc = sqlite3_step(stmtRetrieve);
    while(rc == SQLITE_ROW)
    {
        map_ids.push_back(sqlite3_column_int(stmtRetrieve, 0));
        rc = sqlite3_step(stmtRetrieve);
    }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    return map_ids;
}


//################### METHODS FOR TABLE LINKPOINT CANDIDATES #################

void DBDriver::create_table_linkpoint_candidates(std::string url)
{
   sqlite3 *conn = DBDriver::open_database(url);
   char *sql;
   char *zErrMsg = 0;
   int rc;

    /* Create SQL statement */
   sql = "CREATE TABLE LINKPOINTCANDIDATES("  \
      "ID               INT   PRIMARY KEY     NOT NULL," \
      "MAP_ID           INT                   NOT NULL," \
      "POS_X            REAL                  NOT NULL," \
      "POS_Y            REAL                  NOT NULL," \
      "THETA            REAL                  NOT NULL," \
      "POS_X_WORLD      REAL                  NOT NULL," \
      "POS_Y_WORLD      REAL                  NOT NULL," \
      "THETA_WORLD      REAL                  NOT NULL," \
      "POINTCLOUD_PATH  TEXT                  NOT NULL," \
      "IMAGE            BLOB                  NOT NULL," \
      "DIFFICULTY       INT                           ," \
      "ENVIRONMENT      INT                           );";

   /* Execute SQL statement */
   rc = sqlite3_exec(conn, sql, NULL, 0, &zErrMsg);

   if( rc != SQLITE_OK ){
      fprintf(stderr, "SQL error: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
   } else {
      fprintf(stdout, "Table linkpoint cadidates created successfully\n");
   }
   sqlite3_close(conn);
}

bool DBDriver::insert_linkpoint_candidate(std::string url, LinkPoint lp, cv::Mat img, std::string pointcloud_path)
{
    sqlite3 *conn = DBDriver::open_database(url);
    int rc;

    sqlite3_stmt* stmtInsert = 0;

    /* Create SQL statement */
    std::stringstream sql_ss;

    sql_ss << "INSERT INTO LINKPOINTCANDIDATES (ID,MAP_ID,POS_X,POS_Y, THETA, POS_X_WORLD, POS_Y_WORLD, THETA_WORLD, POINTCLOUD_PATH, IMAGE, DIFFICULTY,ENVIRONMENT) "
              << "VALUES ("
              << lp.get_id() << ","
              << lp.get_map_id(0) << ","
              << lp.get_pos_x(0) << ","
              << lp.get_pos_y(0) << ","
              << lp.get_theta(0) << ","
              << lp.get_pos_x(2) << ","
              << lp.get_pos_y(2) << ","
              << lp.get_theta(2) << ","
              << "'" << pointcloud_path.c_str() << "', "
              << "?, "
              << lp.get_difficulty() << ","
              << lp.get_environment()
              <<");";

    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    //prepare sql statement
    sqlite3_prepare_v2(conn, sql, -1, &stmtInsert, 0);

    //bind blobs for image to statement
    cv::Mat imageCompressed = DBDriver::compress_image(img);
    std::cout << "Compressed Image:\n";
    std::cout << "Cols:" << imageCompressed.cols << "\n";
    std::cout << "Rows:" << imageCompressed.rows << "\n";
    std::cout << "data:" << imageCompressed.data << "\n\n";
    rc = sqlite3_bind_blob(stmtInsert, 1, imageCompressed.data, imageCompressed.cols, SQLITE_STATIC);

    //execute statement
    rc = sqlite3_step(stmtInsert);
    if (rc != SQLITE_DONE){
        std::cerr << "execution failed!" << sqlite3_errmsg(conn) << std::endl;
        sqlite3_finalize(stmtInsert);
        sqlite3_close(conn);
        return false;
     }
    sqlite3_finalize(stmtInsert);
    sqlite3_close(conn);
    return true;
}

LinkPoint DBDriver::get_linkpoint_candidate(std::string url, int id)
{
    LinkPoint lp;
    lp.set_id(id);
    lp.set_map_id(DBDriver::get_db_property_int(url, id, "MAP_ID", "LINKPOINTCANDIDATES"), 0);
    lp.set_pos_x(DBDriver::get_db_property_double(url, id, "POS_X", "LINKPOINTCANDIDATES"), 0);
    lp.set_pos_y(DBDriver::get_db_property_double(url, id, "POS_Y", "LINKPOINTCANDIDATES"), 0);
    lp.set_theta(DBDriver::get_db_property_double(url, id, "THETA", "LINKPOINTCANDIDATES"), 0);
    lp.set_pos_x(DBDriver::get_db_property_double(url, id, "POS_X_WORLD", "LINKPOINTCANDIDATES"), 2);
    lp.set_pos_y(DBDriver::get_db_property_double(url, id, "POS_Y_WORLD", "LINKPOINTCANDIDATES"), 2);
    lp.set_theta(DBDriver::get_db_property_double(url, id, "THETA_WORLD", "LINKPOINTCANDIDATES"), 2);
    lp.set_difficulty(DBDriver::get_db_property_int(url, id, "DIFFICULTY", "LINKPOINTCANDIDATES"));
    lp.set_environment(DBDriver::get_db_property_int(url, id, "ENVIRONMENT", "LINKPOINTCANDIDATES"));
    return lp;
}

cv::Mat DBDriver::get_linkpoint_candidate_image(std::string url, int id)
{
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;
    const char* data = "Callback function called";
    cv::Mat image;

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT IMAGE FROM LINKPOINTCANDIDATES WHERE ID = " << id <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    if(sqlite3_step(stmtRetrieve) == SQLITE_ROW){
        const void * data = 0;
        int dataSize = 0;

        data = sqlite3_column_blob(stmtRetrieve, 0);
        dataSize = sqlite3_column_bytes(stmtRetrieve, 0);

        if(dataSize>0 && data){
            image = rtabmap::uncompressImage(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
        }
        else{
            ROS_ERROR("Error retrieving image from database!");
        }
    }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    return image;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DBDriver::get_linkpoint_candidate_pointcloud(std::string url, int id){
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;
    const char* data = "Callback function called";
    cv::Mat cv_mat_pointloud;

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT POINTCLOUD FROM LINKPOINTCANDIDATES WHERE ID = " << id <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    if(sqlite3_step(stmtRetrieve) == SQLITE_ROW){
        const void * data = 0;
        int dataSize = 0;

        data = sqlite3_column_blob(stmtRetrieve, 0);
        dataSize = sqlite3_column_bytes(stmtRetrieve, 0);

        if(dataSize>0 && data){
            cv_mat_pointloud = rtabmap::uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
	    ROS_INFO("finished decompression of pointcloud");
        }
        else{
            ROS_ERROR("Error retrieving image from database!");
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr = DBDriver::cv_mat_to_pcl(cv_mat_pointloud);
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    return pointcloud_ptr;
}

void DBDriver::delete_linkpoint_candidate(std::string url, int id){
    sqlite3 *conn = DBDriver::open_database(url);
    char *zErrMsg = 0;
    int rc;

     /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "DELETE FROM LINKPOINTCANDIDATES WHERE ID = " << id <<";";

    std::string  str(sql_ss.str());
    const char *sql = str.c_str();
    /* Execute SQL statement */
    rc = sqlite3_exec(conn, sql, NULL, 0, &zErrMsg);

    if( rc != SQLITE_OK ){
       fprintf(stderr, "SQL error: %s\n", zErrMsg);
       sqlite3_free(zErrMsg);
    } else {
       fprintf(stdout, "Deleted linkpoint candidate with id %i!!!\n", id);
    }
    sqlite3_close(conn);
}

std::vector<int> DBDriver::get_lp_candidate_ids(std::string url){
    sqlite3 *conn = DBDriver::open_database(url);
    int rc;

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT " << "ID" << " FROM " << "LINKPOINTCANDIDATES" <<";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtRetrieve = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtRetrieve, 0);

    std::vector<int> lp_candidate_ids;
    rc = sqlite3_step(stmtRetrieve);
    while(rc == SQLITE_ROW)
    {
        lp_candidate_ids.push_back(sqlite3_column_int(stmtRetrieve, 0));
        rc = sqlite3_step(stmtRetrieve);
    }
    sqlite3_finalize(stmtRetrieve);
    sqlite3_close(conn);
    return lp_candidate_ids;
}


//###### UTILITY METHODS #############

cv::Mat DBDriver::compress_image(cv::Mat image){
    std::vector<unsigned char> bytes;
    if(!image.empty())
    {
            if(image.type() == CV_32FC1)
            {
                    //save in 8bits-4channel
                    cv::Mat bgra(image.size(), CV_8UC4, image.data);
                    cv::imencode(".png", bgra, bytes);
            }
            else
            {
                    cv::imencode(".png", image, bytes);
            }
    }

    if(bytes.size())
        {
            //debug
            std::cout << "after compression before insert in sql => cv::Mat pointcloud[0].z = " << image.at<double>(2,0) << "\n";

            return cv::Mat(1, (int)bytes.size(), CV_8UC1, bytes.data()).clone();
        }
    return cv::Mat();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DBDriver::cv_mat_to_pcl(cv::Mat openCVpointcloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    //debug
    //std::cout << "cv::Mat pointcloud[0].z = " << openCVpointcloud.at<double>(2,0);
    ROS_INFO("in function cv_mat_to_pcl");

    for(int i=0;i<openCVpointcloud.cols;i++)
    {
    pcl::PointXYZ point;
    point.x = openCVpointcloud.at<float>(0,i);
    point.y = openCVpointcloud.at<float>(1,i);
    point.z = openCVpointcloud.at<float>(2,i);

    point_cloud_ptr -> points.push_back(point);
    }

    //debug
    ROS_INFO("after for-loop");

    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    //debug
    std::cout << "pcl pointcloud[0] = " << point_cloud_ptr->points[0];

    return point_cloud_ptr;
}

int DBDriver::get_number_of_rows(std::string url, std::string table){
    sqlite3 *conn = DBDriver::open_database(url);
    int rc;
    int cnt_rows = 0;

    /* Create SQL statement */
    std::stringstream sql_ss;
    sql_ss    << "SELECT* FROM " << table.c_str() << ";";
    std::string  str(sql_ss.str());
    const char *sql = str.c_str();

    sqlite3_stmt* stmtCnt = 0;
    sqlite3_prepare_v2(conn, sql, -1, &stmtCnt, 0);

    rc = sqlite3_step(stmtCnt);

    while(rc == SQLITE_ROW)
    {

        cnt_rows++;
        rc = sqlite3_step(stmtCnt);
    }
    sqlite3_finalize(stmtCnt);
    sqlite3_close(conn);
    return cnt_rows;
}

