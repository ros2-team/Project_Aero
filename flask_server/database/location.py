from database.database import get_db_connection

def get_location_by_code(location_code):
    conn = get_db_connection()
    cursor = conn.cursor(dictionary=True)

    sql="""
        select
            location_code,
            location_name,
            pos_x,
            pos_y,
            yaw
        from location 
        where location_code = %s
        """
    cursor.execute(sql,(location_code,))
    result = cursor.fetchone()
    cursor.close()
    conn.close()
    return result