from flask import Flask

# 팀원별 라우트 파일 임포트
from routes.home_routes import home_bp
# from routes.guide_routes import guide_bp   # 나중에 팀원이 완성하면 주석 해제
# from routes.nav_routes import nav_bp       # 나중에 팀원이 완성하면 주석 해제
# from routes.voice_routes import voice_bp   # 나중에 팀원이 완성하면 주석 해제

def create_app():
    app = Flask(__name__)

    # Blueprint 등록 (충돌 방지를 위해 각자의 영역을 설정)
    app.register_blueprint(home_bp)
    
    # 예시: 다른 팀원의 라우트는 url_prefix를 주어 경로 충돌을 막음
    # app.register_blueprint(guide_bp, url_prefix='/guide')
    # app.register_blueprint(nav_bp, url_prefix='/nav')

    return app

if __name__ == '__main__':
    app = create_app()
    app.run(host='0.0.0.0', port=5000, debug=True)