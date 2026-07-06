const moreGuideBtn = document.getElementById("moreGuideBtn");
const finishServiceBtn = document.getElementById("finishServiceBtn");


async function resetNavigationState() {
    const response = await fetch("/api/navigation/reset", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        }
    });

    if (!response.ok) {
        throw new Error(`navigation reset failed: ${response.status}`);
    }

    const data = await response.json();

    if (data.status !== "success") {
        throw new Error(data.message || "navigation reset failed");
    }

    return data;
}


moreGuideBtn.addEventListener("click", async () => {
    try {
        localStorage.removeItem("navigationRoute");

        await resetNavigationState();

        location.href = "/destination";
    } catch (error) {
        console.error(error);
        alert("안내 상태 초기화 중 오류가 발생했습니다.");
    }
});


finishServiceBtn.addEventListener("click", async () => {
    try {
        localStorage.removeItem("navigationRoute");

        await resetNavigationState();

        location.href = "/idle";
    } catch (error) {
        console.error(error);
        alert("안내 종료 처리 중 오류가 발생했습니다.");
    }
});