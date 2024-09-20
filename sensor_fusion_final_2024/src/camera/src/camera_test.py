import cv2

def open_camera(index):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"카메라 {index}를 열 수 없습니다.")
        return None

    # 카메라 해상도와 프레임 속도 설정
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 15)
    
    return cap

def main():
    # 각각의 카메라를 열기 위해 카메라 인덱스를 테스트합니다.
    cap1 = open_camera(0) # left
    cap2 = open_camera(2) # center
    # cap3 = open_camera(4) # center

    # if not cap1 or not cap2 or not cap3:
    if not cap1 or not cap2:
        print("모든 카메라를 열 수 없습니다.")
        if cap1:
            cap1.release()
        if cap2:
            cap2.release()
        # if cap3:
        #     cap3.release()
        return

    while True:
        # 각 카메라에서 프레임 읽기
        ret1, frame1 = cap1.read()
        print("frame1 : ", frame1.shape)

        ret2, frame2 = cap2.read()
        print("frame2 : ", frame2.shape)

        # ret3, frame3 = cap3.read()
        # print("frame3 : ", frame3.shape)

        if not ret1:
            print("카메라 1에서 프레임을 읽을 수 없습니다.")
        if not ret2:
            print("카메라 2에서 프레임을 읽을 수 없습니다.")
        # if not ret3:
        #     print("카메라 3에서 프레임을 읽을 수 없습니다.")

        # 프레임이 None이 아닌 경우에만 표시
        if frame1 is not None:
            cv2.imshow('USB Camera 1', frame1)
        if frame2 is not None:
            cv2.imshow('USB Camera 2', frame2)
        # if frame3 is not None:
        #     cv2.imshow('USB Camera 3', frame3)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 모든 자원을 해제하고 창 닫기
    cap1.release()
    cap2.release()
    # cap3.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
