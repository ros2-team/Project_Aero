
import mysql.connector

conn = mysql.connector.connect(
    host = "localhost",
    user = "projectAR",
    password = "1234",
    database = "projectAR"
)

cursor = conn.cursor()

cursor.execute("select * from location")

rows = cursor.fetchall()

for row in rows:
    print(row)
cursor.close()
conn.close()