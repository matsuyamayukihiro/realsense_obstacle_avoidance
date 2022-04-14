import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import socket
import time

# カメラセットアップ&公正 18.8fps
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

config.enable_device('135222070395')  # テスト用 838212070171　　車体用 135222070395
pipeline.start(config)

# Alignオブジェクト生成
align_to = rs.stream.color
align = rs.align(align_to)
print('ready')

# 変更パラメータ
detection_box = 1  # 検出範囲の調整（最大値は「40」までで設定してね）　値大→ 物体検出する範囲が広くなる

# 障害物検出範囲の指定及び確認用に表示
detection_left_point = 41 - detection_box  # 検出範囲左端
detection_light_point = 399 + detection_box  # 検出範囲右端
detection_upper_point = 41 - detection_box  # 検出範囲上端
detection_under_point = 398 + detection_box  # 検出範囲下端


# 送信側アドレスの設定
# SrcIP = "127.0.0.1"
# 送信側IP
# SrcPort = 11111  # 送信側ポート番号
# SrcAddr = (SrcIP, SrcPort)  # 送信側アドレスをtupleに格納
# 受信側アドレスの設定
# DstIP = "127.0.0.1"
# 受信側IP
# DstPort = 22222  # 受信側ポート番号
# DstAddr = (DstIP, DstPort)  # 受信側アドレスをtupleに格納

def distance_calculation(detframe, detframe_road):  # 白から黒の計算
    im_gray_calc = 0.299 * detframe[:, :, 2] + 0.587 * detframe[:, :, 1] + 0.114 * detframe[:, :, 0]  # 白から黒の値算出
    im_gray_calc = sum(im_gray_calc) / 359.0  # 縦列要素の統合
    im_gray_calc = sum(im_gray_calc) / 360.0  # 横列の統合
    im_gray_calc_road = 0.299 * detframe_road[:, :, 2] + 0.587 * detframe_road[:, :, 1] + 0.114 * detframe_road[:, :, 0]  # 白から黒の値算出
    im_gray_calc_road = sum(im_gray_calc_road) / 80.0  # 縦列要素の統合
    im_gray_calc_road = sum(im_gray_calc_road) / 640.0  # 横列の統合
    return im_gray_calc, im_gray_calc_road


def bamp_check(gray_calc_road, before_gray_calc_road):  # 段差チェック関数
    before_hight = before_gray_calc_road
    now_hight = gray_calc_road
    bamp = abs(before_hight - now_hight)
    if bamp > 28:
        bamp_emagancy = 1
        return bamp_emagancy

    else:
        bamp_emagancy = 0
        return bamp_emagancy


def risk_judgment(im_gray_calc, bamp):  # 距離に応じて3つのモードに分ける　距離情報を正規化
    if (im_gray_calc >= 0) and (im_gray_calc <= 160) or (bamp == 1):  # 右のコードは路面段差検知の条件式 :  # 停止モード
        power_control = 0.0  # 距離情報を正規化
        print('bamp')

    elif (im_gray_calc > 160) and (im_gray_calc <= 240):  # 減速モード
        deceleration_ratio = 255 - im_gray_calc
        power_control = (im_gray_calc / 255) - (deceleration_ratio * 0.0039)  # 減速率  deceleration ratio

    else:  # 通常走行モード
        power_control = 1.0  # 距離情報を正規化   減速率  deceleration ratio

    return power_control


i = 0
# メインループ
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

        img_R = cv2.rectangle(color_image_R, (detection_left_point, detection_upper_point), (detection_light_point, detection_under_point),
                              (0, 255, 0), 3)
        img_R = cv2.rectangle(img_R, (0, 400), (640, 480), (255, 0, 0), 3)

        # 距離に基づくヒートマップ画像グレースケール化して表現
        depth_colormap_gly = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_R, alpha=0.08), cv2.COLORMAP_BONE)
        cv2.namedWindow('RealSense_rgb', cv2.WINDOW_AUTOSIZE)  # BGR画像
        cv2.imshow('RealSense_rgb', img_R)  # BGR画像
        cv2.namedWindow('RealSense_depth', cv2.WINDOW_AUTOSIZE)  # デプス画像
        cv2.imshow('RealSense_depth', depth_colormap_gly)  # デプス画像

        # 特定範囲の指定
        detframe = depth_colormap_gly[detection_upper_point:detection_under_point, detection_left_point:detection_light_point]  # 障害物検知用
        detframe_road = depth_colormap_gly[400:480, 0:640]  # 段差検知用

        # 各ピクセルの距離情報統合、距離に基づく出力パワーの計算
        gray_calc, gray_calc_road = distance_calculation(detframe, detframe_road)  # 距離情報の統合

        if i == 0:
            before_gray_calc_road = gray_calc_road
            pass
        else:
            bamp = bamp_check(gray_calc_road, before_gray_calc_road)  # 路面段差チェック
            before_gray_calc_road = gray_calc_road
            power = risk_judgment(gray_calc, bamp)  # 距離に基づく出力パワーの計算
        # print(power)  # 出力パワーのチェック

        i = 1

        # def data_sending(power_control,SrcAddr, DstAddr):# 送信側プログラム(ローカル用)
        # ソケット作成
        # udpClntSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 引数1 IPv4用 or IPv6用か   引数2 TCP用 or UDP用か
        # udpClntSock.bind(SrcAddr)  # 送信側アドレスでソケットを設定

        # バイナリに変換
        # data = power_control.encode('utf-8')

        # 受信側アドレスに送信
        # udpClntSock.sendto(data, DstAddr)

        cv2.waitKey(10)
        # time.sleep(0.5)

        if cv2.waitKey(1) & 0xff == 27:  # ESCで終了
            cv2.destroyAllWindows()
            break
finally:

    # ストリーミング停止
    pipeline.stop()
