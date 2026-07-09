import os
import sys
import qrcode

# tools 폴더에서 실행해도 상위 프로젝트 모듈을 찾을 수 있게 경로 추가
BASE_DIR = os.path.dirname(
    os.path.dirname(
        os.path.abspath(__file__)
    )
)

sys.path.append(BASE_DIR)

from database.database import get_db_connection


# Flask 서버 주소
# 휴대폰으로 스캔할 거면 127.0.0.1 말고 서버 PC의 실제 IP를 넣어야 함
SERVER_BASE_URL = "http://192.168.0.9:5000"

OUTPUT_DIR = os.path.join(
    BASE_DIR,
    "static",
    "img",
    "qr"
)


def get_locations_from_db():
    connection = get_db_connection()

    try:
        with connection.cursor() as cursor:
            sql = """
                SELECT
                    location_code,
                    location_name
                FROM location
                ORDER BY id
            """

            cursor.execute(sql)
            locations = cursor.fetchall()

            return locations

    finally:
        connection.close()


def create_qr_image(location_code, location_name):
    url = f"{SERVER_BASE_URL}/qrcall?location={location_code}"

    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=12,
        border=4
    )

    qr.add_data(url)
    qr.make(fit=True)

    img = qr.make_image(
        fill_color="black",
        back_color="white"
    )

    filename = f"qr_{location_code}.png"

    output_path = os.path.join(
        OUTPUT_DIR,
        filename
    )

    img.save(output_path)

    print("----------------------------------------")
    print(f"location_name: {location_name}")
    print(f"location_code: {location_code}")
    print(f"url: {url}")
    print(f"created: {output_path}")


def main():
    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )

    locations = get_locations_from_db()

    if not locations:
        print("DB에서 location 데이터를 찾지 못했습니다.")
        return

    for location in locations:
        create_qr_image(
            location[0],
            location[1]
        )

    print("----------------------------------------")
    print("QR 이미지 생성 완료")


if __name__ == "__main__":
    main()