import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import socket
import time

# カメラセットアップ&公正
pipeline = rs.pipeline()  # 配列
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.color, 640, 480, rs.format.z16, 30)  # 白黒
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 白黒

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

config.enable_device('838212070171')  # 左836612070195  838212070171 134322070696
pipeline.start(config)

# Alignオブジェクト生成
align_to = rs.stream.color
align = rs.align(align_to)
print('ready')

# 変更パラメータ
deceleration_ratio = 1.5  # 障害物接近時のブレーキのかかり具合を調整　値（大）→ 減速度合い大　値（小）→ 減速度合い小
detection_box = 1.0   # 検出範囲の調整（最大値は「40」までで設定してね）　値大→ 物体検出する範囲が広くなる

try:
    while True:

        # フレーム待ち(Color & Depth)
        frames_R = pipeline.wait_for_frames()
        # aligned_frames_L = align1.process(frames_L)
        aligned_frames_R = align.process(frames_R)
        # color_frame_L = aligned_frames_L.get_color_frame()
        color_frame_R = aligned_frames_R.get_color_frame()
        #  depth_frame_L = aligned_frames_L.get_depth_frame()
        depth_frame_R = aligned_frames_R.get_depth_frame()
        if not depth_frame_R or not color_frame_R:  # データ取得まで待機
            continue

        # imageをnumpy arrayに
        color_image = np.asanyarray(color_frame_R.get_data())  # RGB
        depth_image = np.asanyarray(depth_frame_R.get_data())  # depth
        color_image_R = cv2.flip(color_image, -1)
        depth_image_R = cv2.flip(depth_image, -1)

        # 障害物検出範囲の指定及び確認用に表示
        detection_left_point = 41 - detection_box  # 検出範囲左
        detection_light_point = 399 + detection_box  # 検出範囲右
        detection_upper_point = 41 - detection_box  # 検出範囲右
        detection_under_point = 199 + detection_box  # 検出範囲右
        img_R = cv2.rectangle(color_image_R, (detection_left_point, detection_upper_point), (detection_light_point, detection_under_point), (0, 255, 0), 3)

        # 距離に基づくヒートマップ画像グレースケール化して表現
        depth_colormap_gly = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_R, alpha=0.08), cv2.COLORMAP_BONE)
        cv2.namedWindow('RealSense_rgb', cv2.WINDOW_AUTOSIZE)  # BGR画像
        cv2.imshow('RealSense_rgb', img_R)  # BGR画像
        cv2.namedWindow('RealSense_depth', cv2.WINDOW_AUTOSIZE)  # デプス画像
        cv2.imshow('RealSense_depth', depth_colormap_gly)  # デプス画像
        im_gray_calc = 0.299 * depth_colormap_gly[:, :, 2] + 0.587 * depth_colormap_gly[:, :, 1] + 0.114 * depth_colormap_gly[:, :, 0]  # 白から黒の値算出
        im_gray_calc = sum(im_gray_calc) / 480.0  # 各チャンネルの合算値計算
        im_gray_calc = sum(im_gray_calc) / 640.0  # R_ch + G_ch + B_ch

        # 距離に応じて3つのモードに分ける
        if (im_gray_calc >= 0) and (im_gray_calc <= 90):
            distance = 0.0  # 距離情報を正規化
            print(distance)

        elif (im_gray_calc > 90) and (im_gray_calc <= 200):
            distance = im_gray_calc / (170.0 * deceleration_ratio)  # 距離情報を正規化   減速率  deceleration ratio
            print(distance)

        else:
            distance = 1.0  # 距離情報を正規化   減速率  deceleration ratio
            print(distance)

        cv2.waitKey(10)
        # time.sleep(3)

        if cv2.waitKey(1) & 0xff == 27:  # ESCで終了
            cv2.destroyAllWindows()
            break
finally:

    # ストリーミング停止
    pipeline.stop()
