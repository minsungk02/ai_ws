import os                           # 파일 및 디렉터리 경로 관리 모듈
import torch                        # PyTorch 텐서 및 연산 라이브러리
import torch.nn as nn               # 신경망 계층 정의를 위한 모듈

class SteeringModel(nn.Module):     # 조향각 회귀를 위한 CNN 모델 정의
    def __init__(self):
        super().__init__()          # 부모 클래스(nn.Module) 초기화

        self.cnn = nn.Sequential(   # 합성곱 신경망(CNN) 계층 정의
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),    # RGB 입력(3채널) → 24채널, 커널 5, 스트라이드 2
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),   # 24채널 → 36채널, 다운샘플링
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),   # 36채널 → 48채널, 다운샘플링
            nn.Conv2d(48, 64, 3, padding=1), nn.ReLU(),  # 48채널 → 64채널, 커널 3, 패딩 유지
            nn.Conv2d(64, 64, 3, padding=1), nn.ReLU(),  # 64채널 → 64채널, 커널 3, 패딩 유지
        )

        with torch.no_grad():                           # 파라미터 업데이트 없이 출력 크기 계산
            dummy = torch.zeros(1, 3, 60, 320)          # 입력 이미지 크기 (배치1, 채널3, 높이60, 너비320)
            out = self.cnn(dummy)                       # CNN 통과
            self.flat_dim = out.view(1, -1).shape[1]    # Flatten 후 차원 크기 저장

        self.fc = nn.Sequential(                       # 완전연결층(FC) 정의
            nn.Flatten(),                              # 2D → 1D 변환
            nn.Linear(self.flat_dim, 100), nn.ReLU(),  # 특징 벡터 → 100차원
            nn.Linear(100, 50), nn.ReLU(),             # 100차원 → 50차원
            nn.Linear(50, 10), nn.ReLU(),              # 50차원 → 10차원
            nn.Linear(10, 1)                           # 최종 출력 (조향각 1값)
        )

    def forward(self, x):              # 순전파 정의
        return self.fc(self.cnn(x))    # CNN 특징 추출 후 FC 통해 출력

def get_unique_train_folder(base="train"):        # 고유 학습 결과 폴더 생성 함수
    os.makedirs(base, exist_ok=True)              # 기본 폴더 생성
    for i in range(1, 1000):                      # exp1 ~ exp999 반복 탐색
        path = os.path.join(base, f"exp{i}")      # exp{i} 경로 지정
        if not os.path.exists(path):              # 경로가 없으면
            os.makedirs(path)                     # 새 폴더 생성
            return path                           # 해당 경로 반환
    return os.path.join(base, "exp999")           # 1000개 이상이면 exp999 반환
