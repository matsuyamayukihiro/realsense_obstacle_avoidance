# First import the library
import socket
import pyrealsense2 as rs
import numpy as np
import cv2

# Create a pipeline
pipeline = rs.pipeline()
# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

config.enable_device('838212070171')
# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()  # 深度スケールをメートルで取得
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 5.5  # 排除する距離パラメータ[m]
clipping_distance = clipping_distance_in_meters / depth_scale  # depth基準作り

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)
c = 0

# Create a pipeline
pipeline1 = rs.pipeline()
# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config1 = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper1 = rs.pipeline_wrapper(pipeline1)
pipeline_profile1 = config1.resolve(pipeline_wrapper1)
device1 = pipeline_profile1.get_device()
device_product_line1 = str(device1.get_info(rs.camera_info.product_line))

found_rgb1 = False
for s in device1.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb1 = True
        break
if not found_rgb1:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line1 == 'L500':
    config1.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

config1.enable_device('912112073474')
# Start streaming
profile1 = pipeline1.start(config1)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor1 = profile1.get_device().first_depth_sensor()  # 深度スケールをメートルで取得
depth_scale1 = depth_sensor1.get_depth_scale()
print("Depth Scale is: ", depth_scale1)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance1 = clipping_distance_in_meters / depth_scale1  # depth基準作り

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to1 = rs.stream.color
align1 = rs.align(align_to1)
c1 = 0

# Streaming loop
try:
    while True:
        # データ取得
        frames_R = pipeline.wait_for_frames()
        frames_L = pipeline1.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames_R = align.process(frames_R)
        aligned_frames_L = align1.process(frames_L)
        # Get aligned frames
        aligned_depth_frame_R = aligned_frames_R.get_depth_frame()  # Dデータ
        color_frame_R = aligned_frames_R.get_color_frame()  # RGBデータ
        aligned_depth_frame_L = aligned_frames_L.get_depth_frame()  # Dデータ
        color_frame_L = aligned_frames_L.get_color_frame()  # RGBデータ

        # Validate that both frames are valid
        if not aligned_depth_frame_R or not color_frame_R or not aligned_depth_frame_L or not color_frame_L:
            continue

        # 処理部分
        depth_image_R = np.asanyarray(aligned_depth_frame_R.get_data())  # 深度カラーマップの配列1ch
        depth_image_L = np.asanyarray(aligned_depth_frame_L.get_data())  # 深度カラーマップの配列1ch
        color_image_R = np.asanyarray(color_frame_R.get_data())  # RGB表示3ch
        color_image_L = np.asanyarray(color_frame_L.get_data())  # RGB表示3ch
        depth_colormap_R = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_R, alpha=0.5),
                                             cv2.COLORMAP_BONE)  # α=透明度 COLORMAP_BONE  疑似的なカラーマップ処理を施す
        depth_colormap_L = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_L, alpha=0.5),
                                             cv2.COLORMAP_BONE)  # α=透明度 COLORMAP_BONE  疑似的なカラーマップ処理を施す

        # 画質変更
        #   h = 320
        #   w = 480
        #   dst_R = cv2.resize(color_image_R, dsize=(w, h))
        #   dst_L = cv2.resize(color_image_L, dsize=(w, h))
        #   dst_depth_R = cv2.resize(depth_image_R, dsize=(w, h))
        #   dst_depth_L = cv2.resize(depth_image_L, dsize=(w, h))
        #   depth_gry_R = cv2.resize(depth_colormap_R, dsize=(w, h))
        #   depth_gry_L = cv2.resize(depth_colormap_L, dsize=(w, h))

        # ぼかし加工。
        for i in range(2):
            average_square_size = 10  # ぼかしパラメータ 大きくする程にぼけていくdef=15
            sigma_color = 5000  # 色空間に関する標準偏差パラメータ  大きくすると色の平滑化範囲を広げるdef=5000
            sigma_metric = 1  # 距離空間に関する標準偏差パラメータ  大きくすると色の平滑化範囲を広げる (d==0の時のみ有効)
            img_bilateral_R = cv2.bilateralFilter(color_image_R, average_square_size, sigma_color,
                                                  sigma_metric)  # Bilateralオペレータを使用して平滑化
            img_bilateral_L = cv2.bilateralFilter(color_image_L, average_square_size, sigma_color,
                                                  sigma_metric)  # Bilateralオペレータを使用して平滑化
        hsv_img_R = cv2.cvtColor(img_bilateral_R, cv2.COLOR_BGR2HSV)  # HSVモデルに変更
        img_gly_R = cv2.cvtColor(depth_colormap_R, cv2.COLOR_BGR2GRAY)  # グレースケール
        hsv_img_L = cv2.cvtColor(img_bilateral_L, cv2.COLOR_BGR2HSV)  # HSVモデルに変更
        img_gly_L = cv2.cvtColor(depth_colormap_L, cv2.COLOR_BGR2GRAY)  # グレースケール

        # 設定以上の段差検知すると、その部分を白塗する
        white_color = 0  # RGBでの黒色
        depth_image_2d_R = np.dstack((depth_image_R, depth_image_R, depth_image_R))  # 配列同士を奥行きで重ねる。RGBに対してdepth情報追加
        bg_removed_R = np.where((depth_image_2d_R > clipping_distance) | (depth_image_2d_R < 0), depth_colormap_R, color_image_R)  # 引数1
        depth_image_2d_L = np.dstack((depth_image_L, depth_image_L, depth_image_L))  # 配列同士を奥行きで重ねる。RGBに対してdepth情報追加
        bg_removed_L = np.where((depth_image_2d_L > clipping_distance1) | (depth_image_2d_L < 0), depth_colormap_L, color_image_L)  # 引数1

        # 2値処理
        lower = (0, 7, 140)  # 色相は赤→黄→緑→水色→青→紫でループする
        upper = (359, 255, 255)
        bin_img_R = cv2.inRange(bg_removed_R, lower, upper)  # 2値化   引数1 画像指定  引数2 下限の色  引数3 上限の色
        contours_R, hierarchy_R = cv2.findContours(bin_img_R, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 輪郭検出
        contours_R = list(filter(lambda x: cv2.contourArea(x) > 10000, contours_R))  # 小さい輪郭は誤検出として削除する
        bin_img_L = cv2.inRange(bg_removed_L, lower, upper)  # 2値化   引数1 画像指定  引数2 下限の色  引数3 上限の色
        contours_L, hierarchy_L = cv2.findContours(bin_img_L, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 輪郭検出
        contours_L = list(filter(lambda x: cv2.contourArea(x) > 10000, contours_L))  # 小さい輪郭は誤検出として削除する

        # Show images
        cv2.namedWindow('Right', cv2.WINDOW_AUTOSIZE)  # 条件式  #引数2 条件合致時の置き換え #引数3 条件不一致時の置き換え
        cv2.imshow('Right', bin_img_R)  # 輪郭処理
        cv2.namedWindow('Left', cv2.WINDOW_AUTOSIZE)  # 条件式  #引数2 条件合致時の置き換え #引数3 条件不一致時の置き換え
        cv2.imshow('Left', bin_img_L)  # 輪郭処理

        xr = 440
        xl = 200
        yl1 = 10
        yl2 = 30
        # 右側判定
        if 0 == bin_img_R[yl1, xl].all() or 0 == bin_img_R[yl1, xr].all():  # 前判定ゾーンで監視
            dataR = 11  # 止まるモード

        elif 0 == bin_img_R[yl1, xr].all() or 0 == bin_img_R[yl2, xr].all():  # 引数1 y座標  引数2 x座標
            dataR = 22  # 左モード

        elif 0 == bin_img_R[yl1, xl].all() or 0 == bin_img_R[yl2, xl].all():  # 左判定ゾーンで監視
            dataR = 33  # "右モード

        else:
            dataR = 44  # 直進モード

        # 左側判定
        if 0 == bin_img_L[yl1, xl].all() or 0 == bin_img_L[yl1, xr].all():  # 前判定ゾーンで監視
            dataL = 11  # 止まるモード

        elif 0 == bin_img_L[yl1, xr].all() or 0 == bin_img_L[yl2, xr].all():  # 引数1 y座標  引数2 x座標
            dataL = 22  # 左モード

        elif 0 == bin_img_L[yl1, xl].all() or 0 == bin_img_L[yl2, xl].all():  # 左判定ゾーンで監視
            dataL = 33  # 右モード

        else:
            dataL = 44  # 直進

        # 総合判定
        if dataR == 11 and dataL == 11:
            print("stop")
            data = 'stop'

        elif dataR == 22 and dataL == 22:  # 引数1 y座標  引数2 x座標
            print("turn Left")
            data = 'turn Left'

        elif dataR == 33 and dataL == 33:  # 左判定ゾーンで監視
            print("turn Right")
            data = 'turn Right'

        elif dataR == 44 and dataL == 44:
            print("keep")
            data = 'keep'

        ##################
        # 送信側プログラム(ローカル用)#
        ##################

        #       # 送信側アドレスの設定
        #       SrcIP = "127.0.0.1"
        #       #      送信側IP
        #       SrcPort = 11111  # 送信側ポート番号
        #       SrcAddr = (SrcIP, SrcPort)  # 送信側アドレスをtupleに格納
        #
        #       # 受信側アドレスの設定
        #       DstIP = "127.0.0.1"
        #       # 受信側IP
        #       DstPort = 22222  # 受信側ポート番号
        #       DstAddr = (DstIP, DstPort)  # 受信側アドレスをtupleに格納
        #
        #       # ソケット作成
        #       udpClntSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 引数1 IPv4用 or IPv6用か   引数2 TCP用 or UDP用か
        #       udpClntSock.bind(SrcAddr)  # 送信側アドレスでソケットを設定
        #
        #       # バイナリに変換
        #       data = data.encode('utf-8')
        #
        #       # 受信側アドレスに送信
        #       udpClntSock.sendto(data, DstAddr)

        # Stop streaming
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:

    pipeline1.stop()
