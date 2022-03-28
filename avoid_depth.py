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

config.enable_device('135222070395')  # 838212070171
pipeline.start(config)

# Alignオブジェクト生成
align_to = rs.stream.color
align = rs.align(align_to)
print('ready')

# 変更パラメータ
deceleration_ratio = 1.0  # 障害物接近時のブレーキのかかり具合を調整　値（大）→ 減速度合い大　値（小）→ 減速度合い小
detection_box = 1  # 検出範囲の調整（最大値は「40」までで設定してね）　値大→ 物体検出する範囲が広くなる

# 障害物検出範囲の指定及び確認用に表示
detection_left_point = 41 - detection_box  # 検出範囲左
detection_light_point = 399 + detection_box  # 検出範囲右
detection_upper_point = 41 - detection_box  # 検出範囲右
detection_under_point = 398 + detection_box  # 検出範囲右

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

        # print('x差分')
        # print(detection_light_point - detection_left_point)
        # print('\ny差分')
        # print(detection_under_point - detection_upper_point)

        img_R = cv2.rectangle(color_image_R, (detection_left_point, detection_upper_point), (detection_light_point, detection_under_point),
                              (0, 255, 0), 3)

        img_R = cv2.rectangle(color_image_R, (0, 400), (640, 480),
                              (255, 0, 0), 3)

        # 距離に基づくヒートマップ画像グレースケール化して表現
        depth_colormap_gly = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_R, alpha=0.08), cv2.COLORMAP_BONE)
        cv2.namedWindow('RealSense_rgb', cv2.WINDOW_AUTOSIZE)  # BGR画像
        cv2.imshow('RealSense_rgb', img_R)  # BGR画像
        cv2.namedWindow('RealSense_depth', cv2.WINDOW_AUTOSIZE)  # デプス画像
        cv2.imshow('RealSense_depth', depth_colormap_gly)  # デプス画像

        # 特定範囲の指定
        detframe = depth_colormap_gly[detection_upper_point:detection_under_point, detection_left_point:detection_light_point]  # 障害物検知用
        detframe_road = depth_colormap_gly[400:480, 0:640]  # 段差検知用

        # 白から黒の計算
        im_gray_calc = 0.299 * detframe[:, :, 2] + 0.587 * detframe[:, :, 1] + 0.114 * detframe[:, :, 0]  # 白から黒の値算出
        im_gray_calc = sum(im_gray_calc) / 359.0  # 縦列要素の統合
        im_gray_calc = sum(im_gray_calc) / 360.0  # 横列の統合

        im_gray_calc_road = 0.299 * detframe_road[:, :, 2] + 0.587 * detframe_road[:, :, 1] + 0.114 * detframe_road[:, :, 0]  # 白から黒の値算出
        im_gray_calc_road = sum(im_gray_calc_road) / 80.0  # 縦列要素の統合
        im_gray_calc_road = sum(im_gray_calc_road) / 640.0  # 横列の統合
        #print(im_gray_calc_road)  # 距離計測値の表示

        # 距離に応じて3つのモードに分ける　距離情報を正規化
        if (im_gray_calc >= 0) and (im_gray_calc <= 120) or (im_gray_calc_road <= 100) or (im_gray_calc_road >= 160):  # 右のコードは路面段差検知の条件式 :  # 停止モード
            power_control = 0.0  # 距離情報を正規化
            print(power_control)

        elif (im_gray_calc > 120) and (im_gray_calc <= 220):  # 減速モード
            power_control = im_gray_calc / (256.0 * deceleration_ratio)  # 減速率  deceleration ratio
            print(power_control)

        else:  # 通常走行モード
            power_control = 1.0  # 距離情報を正規化   減速率  deceleration ratio
            print(power_control)

        cv2.waitKey(10)
        # time.sleep(0.5)

        if cv2.waitKey(1) & 0xff == 27:  # ESCで終了
            cv2.destroyAllWindows()
            break
finally:

    # ストリーミング停止
    pipeline.stop()
