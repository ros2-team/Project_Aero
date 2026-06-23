// Home 전용 다국어 데이터 (공통 데이터 외에 Home에서만 쓰이는 문구)
const airportHomeLangData = {
    ko: { prompt_1: "길을 찾고 계신가요?", prompt_2: "원하는 장소까지 안내해 드립니다.", btn_txt: "길안내 시작" },
    en: { prompt_1: "Looking for directions?", prompt_2: "We will guide you to your destination.", btn_txt: "Start Navigation" },
    ja: { prompt_1: "道をお探しですか？", prompt_2: "ご希望の場所までご案内します。", btn_txt: "案内開始" },
    zh: { prompt_1: "您在找路吗？", prompt_2: "我们将为您导航至目的地。", btn_txt: "开始导航" }
};

let airportHomeVoiceActive = false;

// 실시간 시계 업데이트 함수
function airportHomeUpdateClock() {
    const clockElement = document.getElementById('airport_home_live_clock');
    if (!clockElement) return;
    const now = new Date();
    const timeString = now.toTimeString().split(' ')[0]; // HH:MM:SS 포맷 추출
    clockElement.textContent = timeString;
}

// 다국어 렌더링 함수
function airportHomeRenderLanguage(langCode) {
    const fadeElements = document.querySelectorAll('.airport_home_lang_text');
    
    // 페이드 아웃
    fadeElements.forEach(el => el.classList.add('airport_home_fade_out'));

    setTimeout(() => {
        // 공통 문구 매핑 (common.js에 정의된 데이터 활용)
        document.getElementById('airport_home_brand_txt').textContent = airportRobotLangData[langCode].brand;
        
        // Home 전용 문구 매핑
        document.getElementById('airport_home_prompt_1').textContent = airportHomeLangData[langCode].prompt_1;
        document.getElementById('airport_home_prompt_2').textContent = airportHomeLangData[langCode].prompt_2;
        document.getElementById('airport_home_btn_txt').textContent = airportHomeLangData[langCode].btn_txt;

        // 음성 상태 분기 문구 매핑
        const voiceState = airportHomeVoiceActive ? 'voice_on' : 'voice_off';
        document.getElementById('airport_home_voice_txt').textContent = airportRobotLangData[langCode][voiceState];

        // 버튼 UI 액티브 상태 변경
        document.querySelectorAll('.airport_home_lang_btn').forEach(btn => {
            if (btn.getAttribute('data-lang') === langCode) {
                btn.classList.add('active');
            } else {
                btn.classList.remove('active');
            }
        });

        // 페이드 인 복구
        fadeElements.forEach(el => el.classList.remove('airport_home_fade_out'));
    }, 200);
}

// 초기화 바인딩
document.addEventListener('DOMContentLoaded', () => {
    // 1. 시계 초기화
    airportHomeUpdateClock();
    setInterval(airportHomeUpdateClock, 1000);

    // 2. 초기 언어 로드 (common.js 유틸 활용)
    const initialLang = airportCommonGetLanguage();
    airportHomeRenderLanguage(initialLang);

    // 3. 언어 버튼 클릭 이벤트 등록
    document.querySelectorAll('.airport_home_lang_btn').forEach(btn => {
        btn.addEventListener('click', (e) => {
            const targetLang = e.currentTarget.getAttribute('data-lang');
            if (targetLang === airportCommonGetLanguage()) return;
            
            // common.js의 글로벌 상태 변경 함수 호출
            airportCommonSetLanguage(targetLang);
        });
    });

    // 4. 길안내 시작 이동 트랜잭션 (data-url 속성을 이용한 url_for 바인딩)
    const btnStart = document.getElementById('airport_home_btn_start');
    if (btnStart) {
        btnStart.addEventListener('click', () => {
            const targetUrl = btnStart.getAttribute('data-url');
            window.location.href = targetUrl;
        });
    }

    // 5. 음성 안내 버튼 토글
    const btnVoice = document.getElementById('airport_home_btn_voice');
    if (btnVoice) {
        btnVoice.addEventListener('click', () => {
            airportHomeVoiceActive = !airportHomeVoiceActive;
            const icon = btnVoice.querySelector('i');
            
            if (airportHomeVoiceActive) {
                btnVoice.classList.add('active');
                icon.className = 'fa-solid fa-volume-high';
            } else {
                btnVoice.classList.remove('active');
                icon.className = 'fa-solid fa-volume-xmark';
            }
            
            // 상태 변경 후 언어 렌더링 트리거 (현재 언어 유지하며 텍스트만 업데이트)
            airportHomeRenderLanguage(airportCommonGetLanguage());
        });
    }
});

// 외부(다른 모듈)에서 언어가 변경되었을 때 이벤트 감지 후 Home 텍스트 갱신
document.addEventListener('airportLanguageChanged', (e) => {
    airportHomeRenderLanguage(e.detail);
});