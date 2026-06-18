// 공통 localStorage 키 규격화 (절대 중복 불가)
const AIRPORT_ROBOT_STORAGE_KEY = 'airport_robot_selected_language';

// 공통 다국어 데이터 딕셔너리
const airportRobotLangData = {
    ko: { brand: "AIRPORT GUIDE ROBOT", voice_on: "음성 안내 진행 중", voice_off: "음성 안내" },
    en: { brand: "AIRPORT GUIDE ROBOT", voice_on: "Voice Assisting", voice_off: "Voice Assistance" },
    ja: { brand: "AIRPORT GUIDE ROBOT", voice_on: "音声案内中", voice_off: "音声案内" },
    zh: { brand: "AIRPORT GUIDE ROBOT", voice_on: "语音导航中", voice_off: "语音导航" }
};

// 언어 설정 함수 (전역)
function airportCommonSetLanguage(langCode) {
    if (!airportRobotLangData[langCode]) langCode = 'ko';
    localStorage.setItem(AIRPORT_ROBOT_STORAGE_KEY, langCode);
    
    // JS 이벤트 버스를 통해 다른 모든 JS 파일(페이지)에 언어 변경 알림 발송
    const langEvent = new CustomEvent('airportLanguageChanged', { detail: langCode });
    document.dispatchEvent(langEvent);
}

// 현재 언어 가져오기 함수
function airportCommonGetLanguage() {
    return localStorage.getItem(AIRPORT_ROBOT_STORAGE_KEY) || 'ko';
}