import matplotlib.pyplot as plt
import math
import numpy as np
from numpy import linalg as LA

# グラフの設定を関数化
def setup_axes(ax):
    ax.set_xlim(-1.0, 1.0)  # X1軸の範囲
    ax.set_ylim(-1.0, 1.0)  # X2軸の範囲
    ax.axhline(0, color='black', linewidth=1)  # 水平線
    ax.axvline(0, color='black', linewidth=1)  # 垂直線
    ax.set_aspect('equal')
    ax.grid()

# 初期設定
fig, ax = plt.subplots(figsize=(8, 8))
setup_axes(ax)

# 行列の桁数を設定（小数点以下3桁）
np.set_printoptions(suppress=True, precision=3)

# 角度を0<=radian<2πに正規化する
def normalize_radian(radian):
    while radian < 0:
        radian += 2*math.pi

    return radian % (2 * math.pi)

# 逆運動学プログラム
def check_condition(xe1, xe2, link_length):
    distance = math.sqrt(xe1**2 + xe2**2)
    return not (distance > (link_length * 2) or distance <= 0)

def inverse_kinematics(xe1, xe2, link_length):
    r = math.sqrt((xe1**2) + (xe2**2))
    if r < 1e-6:  
        return 0.0, 0.0  
    fai1 = math.acos(r**2 / (2 * link_length * r))
    fai2 = math.acos((2 * link_length**2 - r**2) / (2 * link_length**2))
    theta1 = math.atan2(xe2, xe1) + fai1
    relative_theta2 = (math.pi - fai2)
    abs_theta2 = theta1 - relative_theta2
    return theta1, abs_theta2

# 差の絶対値を比較する関数
def angle_difference(angle1, angle2):
    diff = abs(normalize_radian(angle1 - angle2))
    # 差がπを超える場合、360度を引く（小さい方の差を取る）
    return min(diff, 2 * math.pi - diff)

#角度の許容誤差
tolerance = 0.001

# リンクの長さ(単位は[m])
length = 0.25
#提示力などのベクトルの大きさを調整する変数
scale = 0.02

#トルクの設定
 #トルクの大きさ(絶対値)
tau1_base = 4.0  # トルク τ1  (Nm)
tau2_base = 4.0  # トルク τ2  (Nm)
 #トルクの初期設定
tau1 = 0
tau2 = 0

# マウス座標を初期化
mouse_x1, mouse_x2 = 0, 0

# マウスの動きに連動する関数
def on_mouse_move(event):
    global mouse_x1, mouse_x2
    if event.xdata is not None and event.ydata is not None:
        mouse_x1, mouse_x2 = event.xdata, event.ydata  # マウス位置を取得

# イベントリスナーを追加
fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)

# 外力方向の設定
direction_deg = 45
direction = normalize_radian(np.radians(direction_deg))
speed = 0.75  #外力方向を示すベクトルの大きさ（描画にのみ関係する）
direction_vector = [speed * math.cos(direction), speed * math.sin(direction)]
arrow_scale = 0.20 #外力方向を表示する大きさを決める変数
arrow_dx1 = arrow_scale * direction_vector[0]
arrow_dx2 = arrow_scale * direction_vector[1]
# 外力方向と逆方向の角度を計算
direction_reverse = (direction + np.pi)
direction_reverse_vector = [speed * math.cos(direction_reverse), speed * math.sin(direction_reverse)]
reverse_dx1 = arrow_scale * direction_reverse_vector[0]
reverse_dx2 = arrow_scale * direction_reverse_vector[1]

#描画するトルクの判定を行うエリアのフラグ
area_flag = False
#反対側の力が提示できるか判定するフラグ
reverse_flag = None
reverse_vector = [0,0]

#直線を引くプログラム（描画するわけではない）
def draw_large_line(start_point, angle_rad):
    angle_rad = angle_rad % (2 * np.pi)
    
    #start_pointは(x, y)のタプルになる。        
    direction_x1 = np.cos(angle_rad)
    direction_x2 = np.sin(angle_rad)
    
    x_start, y_start = start_point
    t = 100  #線の長さ
    x, y = x_start + t * direction_x1, y_start + t * direction_x2
    #return np.array([x,y])#端点を出力
    return (x,y)

def line_intersection(p1, p2, q1, q2):
    """
    線分p1-p2とq1-q2の交点を計算します。
    
    p1, p2: 線分1の端点 (x, y) のタプル
    q1, q2: 線分2の端点 (x, y) のタプル
    戻り値: 交点 (x, y) または None (交差しない場合)
    """
    # 線分をパラメトリック形式にする
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = q1
    x4, y4 = q2

    # 行列形式で交点を計算
    A = np.array([[x2 - x1, x3 - x4],
                  [y2 - y1, y3 - y4]])
    b = np.array([x3 - x1, y3 - y1])

    if np.linalg.det(A) == 0:
        return None  # 平行で交点がない場合
    t, u = np.linalg.solve(A, b)

    # t, uが0~1の範囲内であれば交点は線分内
    if 0 <= t <= 1 and 0 <= u <= 1:
        intersection = [x1 + t * (x2 - x1), y1 + t * (y2 - y1)]
        return intersection
    return None  # 線分上に交点がない場合


#x軸の右側とベクトルの角度を求める関数
def Angle_derivation(vector):
    x_axis = np.array([1 ,0])

    #ベクトルの内積
    dot_product = np.dot(x_axis, vector)

    abs_axis = np.linalg.norm(x_axis)
    vector_axis = np.linalg.norm(vector)

    #角度
    angle = np.arccos(dot_product / (abs_axis * vector_axis))

    #外積を計算して角度の取り方を調整
    cross_product = np.cross(x_axis, vector)
    
    if cross_product < 0:
        angle = 2*np.pi - angle
    
    return angle

#----------------------------------------------------------------------------------------#

# メインループ
while plt.fignum_exists(fig.number):
    update_flag = check_condition(mouse_x1, mouse_x2, length)

    if update_flag:
        # 既存の描画をクリア
        ax.cla()
        setup_axes(ax)

        # リンクと角度の計算
        theta1, theta2 = inverse_kinematics(mouse_x1, mouse_x2, length)
        theta1 = normalize_radian(theta1)
        theta2 = normalize_radian(theta2)

        x1a = length * math.cos(theta1)
        x2a = length * math.sin(theta1)

        x1b = length * math.cos(theta2)
        x2b = length * math.sin(theta2)

        x1c = x1b + x1a
        x2c = x2b + x2a

        #座標を回転させる処理
        vector1 = np.array([mouse_x1,mouse_x2])#原点と把持部を結ぶ線のベクトル
        vector2 = np.array([0,mouse_x2])#Y軸上に垂線を引いたときの交点と原点を結ぶ線のベクトル

        if np.linalg.norm(vector1) == 0 or np.linalg.norm(vector2) == 0:
            cos_phi = np.nan
        else:
            cos_phi = np.dot(vector1,vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
            
        if np.isnan(cos_phi):
            abs_phi = np.nan
        else:
            abs_phi = np.arccos(np.clip(cos_phi, -1.0,1.0))

        #第１象限
        if mouse_x2 >= 0 and mouse_x1 >= 0: 
            phi = np.pi - abs_phi
        #第２象限
        elif mouse_x2 >= 0 and mouse_x1 <=0:
            phi = np.pi + abs_phi
        #第３象限
        elif mouse_x1 < 0 and mouse_x2 < 0:
            phi = -1 * abs_phi
        #第４象限
        elif mouse_x2 < 0 and mouse_x1 > 0:
            phi = abs_phi

    #トルクの境界条件に関する処理
        #回転した座標系での境界

        theta_v3_rotate_small = 0
        theta_v4_rotate_small = np.pi / 2
        theta_v3_rotate_large = np.pi
        theta_v4_rotate_large = np.pi * 3 / 2

        #回転前の座標系での境界

        theta_v3_small = normalize_radian(theta_v3_rotate_small + phi)
        theta_v4_small = normalize_radian(theta_v4_rotate_small + phi)
        theta_v3_large = normalize_radian(theta_v3_small + np.pi)
        theta_v4_large = normalize_radian(theta_v4_small + np.pi)

        #トルクの正負判定
        theta_v = normalize_radian(direction - phi)
        
        #print(np.degrees([theta_v3_small,theta_v4_small,theta_v3_large,theta_v4_large,theta_v]))
        #print(np.degrees(phi))

        if angle_difference(theta_v, theta_v4_rotate_small) < tolerance:
            tau1 = -1 * tau1_base
            tau2 = 0
            area_flag = "None"

        elif angle_difference(theta_v, theta_v3_rotate_small) < tolerance:
            tau1 = 0
            tau2 = -1 * tau2_base
            area_flag = "None"

        elif angle_difference(theta_v, theta_v4_rotate_large) < tolerance:
            tau1 = 1 * tau1_base
            tau2 = 0
            area_flag = "None"

        elif angle_difference(theta_v, theta_v3_rotate_large) < tolerance:
            tau1 = 0
            tau2 = 1 * tau1_base
            area_flag = "None"

        elif theta_v3_rotate_small < theta_v < theta_v4_rotate_small:
            tau1 = -1 * tau1_base
            tau2 = -1 * tau2_base
            area_flag = "Red"
        
        elif theta_v4_rotate_large < theta_v < theta_v3_rotate_small + 2*np.pi:
            tau1 = 1 * tau1_base
            tau2 = -1 * tau2_base
            area_flag = "Blue"

        elif theta_v3_rotate_large < theta_v < theta_v4_rotate_large:
            tau1 = 1 * tau1_base
            tau2 = 1 * tau2_base
            area_flag = "Green"
        
        elif theta_v4_rotate_small < theta_v < theta_v3_rotate_large:
            tau1 = -1 * tau1_base
            tau2 = 1 * tau2_base
            area_flag = "Yellow"
        
        else:
            print("エラー")

        #print(area_flag)

        #力の計算
        # τ1 による力の成分を計算 f1がx1,f2がx2のベクトル
        f1_tau1 = -(1 / (length * np.sin((theta1 - theta2) / 2))) * np.cos((theta1 + theta2) / 2) * tau1
        f2_tau1 = -(1 / (length * np.sin((theta1 - theta2) / 2))) * np.sin((theta1 + theta2) / 2) * tau1
        f_tau1 = np.array([f1_tau1,f2_tau1])
        abs_f_tau1 = LA.norm(f_tau1)
        f_tau1_theta = Angle_derivation(f_tau1)

        # τ2 による力の成分を計算
        f1_tau2 = -(1 / ( length * np.cos((theta1 - theta2) / 2))) * np.cos(((theta1 + theta2)/2) - (np.pi / 2)) * tau2
        f2_tau2 = -(1 / ( length * np.cos((theta1 - theta2) / 2))) * np.sin(((theta1 + theta2)/2) - (np.pi / 2)) * tau2
        f_tau2 = np.array([f1_tau2,f2_tau2])
        abs_f_tau2 = LA.norm(f_tau2)
        f_tau2_theta = Angle_derivation(f_tau2)

        # 合成力を計算
        F_total = f_tau1 + f_tau2
        F_total_theta = Angle_derivation(F_total)

        #print(F_total)
        #print(np.degrees([f_tau1_theta,f_tau2_theta,direction_reverse]))

        #リンクの描画
        x1_positions = [0, x1a, mouse_x1]
        x2_positions = [0, x2a, mouse_x2]

        ax.plot(x1_positions, x2_positions, 'o-', lw=3, color='blue')  # リンク
        ax.plot([0, x1b, x1c], [0, x2b, x2c], 'o-', lw=3, color='blue') #リンク
        ax.plot(0, 0, 'ro')  # 原点
        
        #境界の描画
        '''
        line_length = 0.2

        theta_v3_x1positions = [mouse_x1 + (line_length * np.cos(theta_v3_small)), mouse_x1 + (line_length * np.cos(theta_v3_large))]
        theta_v3_x2positions = [mouse_x2 + (line_length * np.sin(theta_v3_small)), mouse_x2 +(line_length * np.sin(theta_v3_large)) ]
        theta_v4_x1positions = [mouse_x1 + (line_length * np.cos(theta_v4_small)), mouse_x1 + (line_length * np.cos(theta_v4_large))]
        theta_v4_x2positions = [mouse_x2 + (line_length * np.sin(theta_v4_small)), mouse_x2 + (line_length * np.sin(theta_v4_large)) ]

        rect_size = 0.15
        theta_v3_x1rectpositions = [mouse_x1 + (rect_size * np.cos(theta_v3_small)), mouse_x1 + (rect_size * np.cos(theta_v3_large))]
        theta_v3_x2rectpositions = [mouse_x2 + (rect_size * np.sin(theta_v3_small)), mouse_x2 +(rect_size * np.sin(theta_v3_large)) ]
        theta_v4_x1rectpositions = [mouse_x1 + (rect_size * np.cos(theta_v4_small)), mouse_x1 + (rect_size * np.cos(theta_v4_large))]
        theta_v4_x2rectpositions = [mouse_x2 + (rect_size * np.sin(theta_v4_small)), mouse_x2 + (rect_size * np.sin(theta_v4_large)) ]
        #print(theta_v1_x1positions,theta_v1_x2positions)
        
        vertices =[
            (theta_v3_x1rectpositions[0] , theta_v3_x2rectpositions[0]),#左上
            (theta_v4_x1rectpositions[0] , theta_v4_x2rectpositions[0]),#右上
            (theta_v4_x1rectpositions[1] , theta_v4_x2rectpositions[1]),#左下
            (theta_v3_x1rectpositions[1] , theta_v3_x2rectpositions[1]) #右下
        ]
        
        triangles = [
            ([mouse_x1, vertices[0][0], vertices[1][0]], 
             [mouse_x2, vertices[0][1], vertices[1][1]], 'red'),  # 上三角形
            ([mouse_x1, vertices[1][0], vertices[3][0]], 
             [mouse_x2, vertices[1][1], vertices[3][1]], 'yellow'),  # 右三角形
            ([mouse_x1, vertices[3][0], vertices[2][0]], 
             [mouse_x2, vertices[3][1], vertices[2][1]], 'green'),  # 下三角形
            ([mouse_x1, vertices[2][0], vertices[0][0]], 
             [mouse_x2, vertices[2][1], vertices[0][1]], 'blue')  # 左三角形
        ]
        
        for (x1_coords, x2_coords, color) in triangles:
            ax.fill(x1_coords, x2_coords, color=color, alpha=0.5)        
        
        #色の分岐（エリアフラグはスライドの色に準拠している）
        if area_flag == "Red":
            area_parameters = triangles[0]
            ax.fill(area_parameters[0], area_parameters[1], area_parameters[2], alpha = 0.5)
        elif area_flag == "Blue":
            area_parameters = triangles[1]
            ax.fill(area_parameters[0], area_parameters[1], area_parameters[2], alpha = 0.5)
        elif area_flag == "Green":
            area_parameters = triangles[2]
            ax.fill(area_parameters[0], area_parameters[1], area_parameters[2], alpha = 0.5)
        elif area_flag == "Yellow":
            area_parameters = triangles[3]
            ax.fill(area_parameters[0], area_parameters[1], area_parameters[2], alpha = 0.5)

        ax.plot(theta_v3_x1positions, theta_v3_x2positions, 'c-', lw=1)#theta_v1を表す線
        ax.plot(theta_v4_x1positions, theta_v4_x2positions, 'm-', lw=1)#theta_v2を表す線
        '''
        #print(tau1,tau2)
        #手先と反対の方向の力の計算
        large_reverse_vector = draw_large_line((mouse_x1, mouse_x2), direction_reverse)
        
        reverse_flag = True
        reverse_intersection = None
        reverse_vector = []
        #許容誤差を指定してから比較する

        if abs(f_tau1_theta - direction_reverse) <= tolerance:
            reverse_vector = f_tau1
        elif abs(f_tau2_theta - direction_reverse) <= tolerance:
            reverse_vector = f_tau2

        elif abs(F_total_theta - direction_reverse) <= tolerance:
            reverse_vector = F_total

        else:

            if reverse_intersection is None:
                reverse_intersection = line_intersection((f1_tau1, f2_tau1),(F_total[0], F_total[1]),(mouse_x1, mouse_x2),large_reverse_vector)
            if reverse_intersection is None:
                reverse_intersection = line_intersection((f1_tau2, f2_tau2),(F_total[0], F_total[1]),(mouse_x1, mouse_x2),large_reverse_vector)
            if reverse_intersection is not None:
                reverse_vector = np.array([reverse_intersection[0] - mouse_x1, reverse_intersection[1] - mouse_x2])

        if len(reverse_vector) == 0:
            reverse_flag = False
            reverse_vector = np.array([speed * math.cos(direction_reverse) - mouse_x1, speed * math.sin(direction_reverse) - mouse_x2])

        #print(reverse_intersection)
        #ベクトルの描画
        
        scale_factor = scale
        #手先速度の方向
        ax.quiver(mouse_x1, mouse_x2, arrow_dx1, arrow_dx2, 
                  angles='xy', scale_units='xy', scale=1, color='red',label='direction movement')

        # τ1 Force
        ax.quiver(mouse_x1, mouse_x2, f_tau1[0] * scale_factor, f_tau1[1] * scale_factor, 
                angles='xy', scale_units='xy', scale=1, color='green', label=f"τ1 Force : {f_tau1}[N]")

        # τ2 Force
        ax.quiver(mouse_x1, mouse_x2, f_tau2[0] * scale_factor, f_tau2[1] * scale_factor, 
                angles='xy', scale_units='xy', scale=1, color='orange', label=f"τ2 Force : {f_tau2}[N]")

        # Reverse Force
        reverse_vector = np.round(reverse_vector, 3)
        if reverse_flag:
            ax.quiver(mouse_x1, mouse_x2, reverse_vector[0] * scale_factor, reverse_vector[1] * scale_factor, 
                    angles='xy', scale_units='xy', scale=1, color='purple', label=f"reverse_force : {reverse_vector}[N]")
        else:
            ax.quiver(mouse_x1, mouse_x2, reverse_dx1 , reverse_dx2, 
                    angles='xy', scale_units='xy', scale=1, color=(0.5, 0.4, 0.7), label="cannot present") 

        #力の提示範囲の描画
        polygon_points = np.array([[mouse_x1,mouse_x2], 
                                   [mouse_x1 + f_tau1[0]*scale , mouse_x2 + f_tau1[1]*scale], 
                                   [mouse_x1 + F_total[0]*scale, mouse_x2 + F_total[1]*scale],
                                   [mouse_x1 + f_tau2[0]*scale, mouse_x2 + f_tau2[1]*scale], 
                                   [mouse_x1,mouse_x2]])
        ax.fill(polygon_points[:, 0], polygon_points[:, 1], color='blue', alpha=0.3, label='Range_Area')
        

    #角度の表示
        ax.text(0.01, -0.12, f"Angle1: {math.degrees(theta1):.2f}°, Angle2: {math.degrees(theta2):.2f}°", 
            transform=ax.transAxes, fontsize=10, color='blue', ha='left')
        
        # 軸ラベルとタイトル
        plt.title("Static Simulation of Link Mechanism")
        #plt.xlabel("Force fe1[N]")
        #plt.ylabel("Force fe2[N]")

        plt.legend()
    plt.pause(0.05)