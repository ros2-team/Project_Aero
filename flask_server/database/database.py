import mysql.connector

def get_db_connection():
    return mysql.connector.connect(
        host = "localhost",
        user = "projectAR",
        password = "1234",
        database = "projectAR"
    )
def get_locations():

    conn = get_db_connection()

    cursor = conn.cursor(dictionary=True)

    cursor.execute("""
        SELECT
            location_code,
            location_name,
            pos_x,
            pos_y,
            yaw
        FROM location
        ORDER BY location_name
    """)
    
    result = cursor.fetchall()

    cursor.close()
    conn.close()

    return result