import mysql.connector

def get_db_connection():
    return mysql.connector.connect(
        host = "localhost",
        user = "projectAR",
        password = "1234",
        database = "projectAR"
    )
def get_location(location_code):
    
    conn = get_db_connection()
    cursor = conn.cursor(dictionary=True)
    
    cursor.execute(
        """select * from location where location_code = %s""",
        (location_code,)
    )
    
    result = cursor.fetchone()
    
    cursor.close()
    conn.close()

    return result