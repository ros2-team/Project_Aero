import cv2
import numpy as np

### hsv 색 비교를 위한 기능 정의 클래스 파일
class HsvTracker:
    def __init__(self):
        ### 타겟 초기화 여부와 타겟의 고유 색상 저장
        self.target_initialized = False
        self.target_hsv = None

        ### 바운딩 박스 중앙 50% 영역(상의 위주)의 HSV 평균값 추출
    def get_core_hsv(self, frame, bbox):

        ### x, y 좌표 int로 변환
        xmin, ymin, xmax, ymax = map(int, bbox)
        
        ### x길이 y길이와 정 중앙 값 cx, cy에 저장
        w, h = xmax - xmin, ymax - ymin
        cx, cy = xmin + w // 2, ymin + h // 2
        
        ### 중앙 50% 영역 계산
        crop_xmin = int(cx - w * 0.25)
        crop_xmax = int(cx + w * 0.25)
        crop_ymin = int(cy - h * 0.25)
        crop_ymax = int(cy + h * 0.25)
        
        ### 관심 영역 저장
        roi = frame[max(0, crop_ymin):crop_ymax, max(0, crop_xmin):crop_xmax]
        
        # 박스가 너무 작거나 화면을 벗어난 예외 처리
        if roi.size == 0:
            return np.array([0, 0, 0])
        
        ### roi 관심영역 안 hsv 값 저장
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        return np.mean(hsv_roi, axis=(0, 1))

        ### H(Hue) 값의 원형 특성(0~180)을 고려한 차이 계산 
    def get_hue_difference(self, hsv1, hsv2):
        diff = abs(hsv1[0] - hsv2[0])
        return min(diff, 180 - diff)

        ### 다수의 바운딩 박스 중 타겟을 선정하여 반환
    def get_target_bbox(self, frame, bboxes):
        if len(bboxes) == 0:
            return None

        ### 타겟이 아직 없을 때: 가장 가까운(면적이 큰) 사람을 타겟으로 지정
        if not self.target_initialized:
            max_area = 0
            closest_bbox = bboxes[0]
            
            for bbox in bboxes:
                area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                if area > max_area:
                    max_area = area
                    closest_bbox = bbox
            
            ### 첫 타겟의 HSV 저장 후 초기화 완료
            self.target_hsv = self.get_core_hsv(frame, closest_bbox)
            self.target_initialized = True
            return closest_bbox

        ### 타겟이 있을 때: 저장된 색상과 가장 비슷한 사람 찾기
        min_diff = float('inf')
        target_bbox = None
        
        for bbox in bboxes:
            current_hsv = self.get_core_hsv(frame, bbox)
            hue_diff = self.get_hue_difference(self.target_hsv, current_hsv)
            
            if hue_diff < min_diff:
                min_diff = hue_diff
                target_bbox = bbox
                
        return target_bbox