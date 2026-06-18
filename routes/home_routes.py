from flask import Blueprint, render_template

# 'home' 이라는 이름의 Blueprint 생성
home_bp = Blueprint('home', __name__)

@home_bp.route('/')
def home_index():
    return render_template('home/home_page.html')

# 병합 전 테스트를 위해 임시로 guide 이동 라우트 생성 (타 팀원 작업 전까지만 사용)
@home_bp.route('/guide')
def temp_guide_index():
    return "<h1 style='color:white; background:#000;'>Guide Page (다른 팀원이 개발할 페이지)</h1>"  