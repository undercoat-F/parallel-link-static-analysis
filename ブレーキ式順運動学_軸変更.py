import matplotlib.pyplot as plt
import math
import numpy as np
#使う関数等の設定----------------------------------------------------------------------------

#順運動学プログラム(絶対角)
def direct_kinematics(theta1,theta2,L):
    #逆正弦関数math.asin()や逆余弦関数math.acos()に入力値が-1から1の範囲外の値を渡すとエラーが発生
    
    # 関節の位置
    xj1 = L * math.cos(theta1) 
    xj2 = L * math.sin(theta1)
    
    # 手先位置
    xe1 = xj1 + L * math.cos(theta2)
    xe2 = xj2 + L * math.sin(theta2)
  
    return xj1, xj2, xe1, xe2

def check_condition(xe1, xe2, link_length):
    distance = math.sqrt(xe1**2 + xe2**2)
    return not (distance > (link_length * 2) or distance <= 0)

# 角度を0<=radian<2πに正規化する
def normalize_radian(radian):
    while radian < 0:
        radian += 2*math.pi

    return radian % (2 * math.pi)

# 差の絶対値を比較する関数
def angle_difference(angle1, angle2):
    diff = abs(normalize_radian(angle1 - angle2))
    # 差がπを超える場合、360度を引く（小さい方の差を取る）
    return min(diff, 2 * math.pi - diff)

#直線を引き、端点を取得する関数
def draw_large_line(start_point, angle_rad):
    angle_rad = angle_rad % (2 * np.pi)
    
    #start_pointは(x1, x2)のタプルになる。        
    direction_x1 = np.cos(angle_rad)
    direction_x2 = np.sin(angle_rad)
    
    x1_start, x2_start = start_point
    t = 100  #線の長さ
    x1, x2 = x1_start + t * direction_x1, x2_start + t * direction_x2
    #return np.array([x1,x2])#端点を出力
    return (x1,x2)

#線分の交点を取得する関数
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

# グラフの設定を関数化
def setup_axes(ax):
    ax.set_xlim(-1.0, 1.0)  # X軸の範囲
    ax.set_ylim(-1.0, 0.5)  # Y軸の範囲
    ax.axhline(0, color='black', linewidth=1)  # 水平線
    ax.axvline(0, color='black', linewidth=1)  # 垂直線
    ax.set_aspect('equal')
    ax.grid()

#メイン----------------------------------------------------------------------------------------#

#pythonの計算の許容誤差
tolerance = 0.05

#角度を打ち込んで取得
print("リンクの角度を theta1[deg] スペース theta2[deg] で打ち込んでください。")
theta1, theta2 = map(float,input().split())
theta1 = normalize_radian(np.radians(theta1))
theta2 = normalize_radian(np.radians(theta2))
print("外力を加える方向を示す角度を thetav[deg] で打ち込んでください")
direction_deg = float(input())
direction = normalize_radian(np.radians(direction_deg))
#トルクの設定
print("ラズベリーパイから出力する電圧[V]を入力してください")
V = float(input())
tau1_base = 0.8 * V # 定数 0.8 は"(減速比*定格トルク)/5"より算出 5は定格トルクが出力される時の電圧
tau2_base = 0.8 * V

# 行列の桁数を設定（小数点以下3桁）
np.set_printoptions(suppress=True, precision=3)

# 外力方向の設定
speed = 1  #外力方向を示すベクトルの大きさ（描画にのみ関係する）
direction_vector = [speed * math.cos(direction), speed * math.sin(direction)]
arrow_scale = 0.27#外力方向を表示する大きさを決める変数
arrow_dx1 = arrow_scale * direction_vector[0]
arrow_dx2 = arrow_scale * direction_vector[1]
# 外力方向と逆方向の角度を計算
direction_reverse = (direction + np.pi)
direction_reverse_vector = [speed * math.cos(direction_reverse), speed * math.sin(direction_reverse)]
reverse_dx1 = arrow_scale * direction_reverse_vector[0]
reverse_dx2 = arrow_scale * direction_reverse_vector[1]

# リンクの長さ(単位は[m])
length = 0.4
#提示力などのベクトルの大きさを調整する変数
scale = 0.04
length_scale = 1

#トルクの初期設定
tau1 = None
tau2 = None

# 把持部位置を初期化
xe1, xe2 = 0, 0

#描画するトルクの判定を行うエリアのフラグ
area_flag = False
#反対側の力が提示できるか判定するフラグ
reverse_flag = None
reverse_vector = []

#------リンクと角度の計算
x1a, x2a, xe1,xe2 = direct_kinematics(theta1, theta2, length)
        
x1b = length/2 * math.cos(theta2)
x2b = length/2 * math.sin(theta2)

x1c = x1b + x1a
x2c = x2b + x2a

update_flag = check_condition(xe1, xe2, length)

#描画用の把持部の座標
xe1scale = xe1 * length_scale
xe2scale = xe2 * length_scale

if update_flag:

    #計算処理-----------------------------------------------------------------------------------------------
    #------座標を回転させる処理
    vector1 = np.array([xe1,xe2])#原点と把持部を結ぶ線のベクトル
    vector2 = np.array([0,xe2])#Y軸上に垂線を引いたときの交点と原点を結ぶ線のベクトル

    if np.linalg.norm(vector1) == 0 or np.linalg.norm(vector2) == 0:
        cos_phi = np.nan
    else:
        cos_phi = np.dot(vector1,vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
        
    if np.isnan(cos_phi):
        abs_phi = np.nan
    else:
        abs_phi = np.arccos(np.clip(cos_phi, -1.0,1.0))
        
    #アームの先端がどの位置にあるかによって回転する角度を調整する
    #第１象限
    if xe2 >= 0 and xe1 >= 0: 
        phi = np.pi - abs_phi
    #第２象限
    elif xe2 >=0 and xe1 <=0:
        phi = np.pi + abs_phi
    #第３象限
    elif xe1 < 0 and xe2 < 0:
        phi = -1 * abs_phi
    #第４象限
    elif xe2 < 0 and xe1 > 0:
        phi = abs_phi

    #境界にあるとき（例外処理）
    if np.isnan(phi):
        phi = np.pi

    #--------トルクの境界条件に関する処理
    #回転した座標系での境界

    theta_v1_rotate_large = normalize_radian(math.atan(-1/(math.tan(theta2 - phi ))))
    theta_v2_rotate_small = normalize_radian(math.atan(-1/(math.tan(theta1 - phi ))))

    theta_v1_rotate_small= normalize_radian(theta_v1_rotate_large - math.pi)
    theta_v2_rotate_large = normalize_radian(theta_v2_rotate_small + math.pi)

    #回転前の座標系での境界
    theta_v1_large = theta_v1_rotate_large + phi
    theta_v2_small = theta_v2_rotate_small + phi
    theta_v1_small = theta_v1_large + math.pi
    theta_v2_large = theta_v2_small + math.pi

    #------トルクの正負判定
    theta_v = normalize_radian(direction - phi)

    if angle_difference(theta_v, theta_v2_rotate_small) < tolerance:
        tau1 = -1 * tau1_base
        tau2 = 0

    elif angle_difference(theta_v, theta_v1_rotate_large) < tolerance:
        tau1 = 0
        tau2 = -1 * tau2_base
    
    elif angle_difference(theta_v, theta_v2_rotate_large) < tolerance:
        tau1 = 1 * tau1_base
        tau2 = 0
    
    elif angle_difference(theta_v, theta_v1_rotate_small) < tolerance:
        tau1 = 0
        tau2 = 1 * tau2_base

    elif theta_v1_rotate_small> theta_v > theta_v2_rotate_small:
        tau1 = -1 * tau1_base
        tau2 = 1 * tau2_base
        area_flag = "Red"

    elif (0 <= theta_v < theta_v2_rotate_small) or (theta_v1_rotate_large < theta_v < 2*math.pi):
        tau1 = -1 * tau1_base
        tau2 = -1 * tau2_base
        area_flag = "Blue"

    elif theta_v2_rotate_large < theta_v < theta_v1_rotate_large:
        tau1 = 1 * tau1_base
        tau2 = -1 * tau2_base
        area_flag = "Green"

    elif theta_v1_rotate_small < theta_v < theta_v2_rotate_large:
        tau1 = 1*tau1_base
        tau2 = 1*tau2_base
        area_flag = "Yellow"

    else:
        print("エラー")

    #------力の計算
    # τ1 による力の成分を計算 f1がx1,f2がx2のベクトル
    f1_tau1 = -(1 / (length * np.sin(theta1 - theta2))) * (np.cos(theta2) * tau1)
    f2_tau1 = -(1 / (length * np.sin(theta1 - theta2))) * (np.sin(theta2) * tau1)
    f_tau1 = np.array([f1_tau1,f2_tau1])
    f_tau1_theta = Angle_derivation(f_tau1)

    # τ2 による力の成分を計算
    f1_tau2 = -(1 / (length * np.sin(theta1 - theta2))) * (-np.cos(theta1) * tau2)
    f2_tau2 = -(1 / (length * np.sin(theta1 - theta2))) * (-np.sin(theta1) * tau2)
    f_tau2 = np.array([f1_tau2,f2_tau2])
    f_tau2_theta = Angle_derivation(f_tau2)

    # 合成力を計算
    F_total = f_tau1 + f_tau2
    F_total_theta = Angle_derivation(F_total)

    #-------外力方向の反対に働く力の計算
    large_reverse_vector = draw_large_line((xe1, xe2), direction_reverse)
    reverse_flag = True
    reverse_intersection = None

    if angle_difference(f_tau1_theta, direction_reverse) <= tolerance:
        reverse_vector = f_tau1
    elif angle_difference(f_tau2_theta, direction_reverse) <= tolerance:
        reverse_vector = f_tau2
    elif angle_difference(F_total_theta, direction_reverse) <= tolerance:
        reverse_vector = F_total
    
    else:
        if reverse_intersection is None:
            reverse_intersection = line_intersection((f_tau1[0]+xe1, f_tau1[1]+xe2),(F_total[0]+xe1, F_total[1]+xe2),(xe1, xe2),large_reverse_vector)
        if reverse_intersection is None:
            reverse_intersection = line_intersection((f_tau2[0]+xe1, f_tau2[1]+xe2),(F_total[0]+xe1, F_total[1]+xe2),(xe1, xe2),large_reverse_vector)
        if reverse_intersection is not None:
            reverse_vector = np.array([reverse_intersection[0] - xe1, reverse_intersection[1] - xe2])            

        if len(reverse_vector) == 0:
            reverse_flag = False
            reverse_vector = np.array([speed * math.cos(direction_reverse) - xe1, speed * math.sin(direction_reverse) - xe2])
            print("OK")
    
    print(np.linalg.norm(reverse_vector))

    #描画処理--------------------------------------------------------------------------
    #初期設定
    fig, ax = plt.subplots(figsize=(8, 8))
    setup_axes(ax)
    #------リンクの描画
    x1_positions = [0, x1a*length_scale, xe1*length_scale]
    x2_positions = [0, x2a*length_scale, xe2*length_scale]

    ax.plot(x1_positions, x2_positions, 'o-', lw=3, color='blue')  # リンク
    ax.plot([0, x1b*length_scale, x1c*length_scale], [0, x2b*length_scale, x2c*length_scale], 'o-', lw=3, color='blue') #リンク
    ax.plot(0, 0, 'ro')  # 原点

    #------ベクトル等の描画
    #外力方向
    ax.quiver(xe1scale, xe2scale, arrow_dx1, arrow_dx2, 
            angles='xy', scale_units='xy', scale=1, color='red',label='direction movement')

    # τ1 Force
    ax.quiver(xe1scale, xe2scale, f_tau1[0] * scale, f_tau1[1] * scale, 
            angles='xy', scale_units='xy', scale=1, color='green', label=f"τ1 Force : {f_tau1}[N]")

    # τ2 Force
    ax.quiver(xe1scale, xe2scale, f_tau2[0] * scale, f_tau2[1] * scale, 
            angles='xy', scale_units='xy', scale=1, color='orange', label=f"τ2 Force : {f_tau2}[N]")

    # Reverse Force
    reverse_vector = np.round(reverse_vector, 3)
    if reverse_flag:
        ax.quiver(xe1scale, xe2scale, reverse_vector[0] * scale, reverse_vector[1] * scale, 
                angles='xy', scale_units='xy', scale=1, color='purple', label=f"reverse_force : {reverse_vector}[N]")
    
    else:
        ax.quiver(xe1scale, xe2scale, reverse_dx1 , reverse_dx2, 
                angles='xy', scale_units='xy', scale=1, color=(0.5, 0.4, 0.7), label="cannot present") 

    #力の提示範囲の描画
    polygon_points = np.array([[xe1scale,xe2scale], 
                               [xe1scale + f_tau1[0]*scale , xe2scale + f_tau1[1]*scale], 
                               [xe1scale + F_total[0]*scale, xe2scale + F_total[1]*scale],
                               [xe1scale + f_tau2[0]*scale, xe2scale + f_tau2[1]*scale], 
                               [xe1scale,xe2scale]])

    ax.fill(polygon_points[:, 0], polygon_points[:, 1], color='blue', alpha=0.3, label='Range_Area')
        
   #角度の表示
    #ax.text(0.01, -0.12, f"Angle1: {math.degrees(theta1):.2f}°, Angle2: {math.degrees(theta2):.2f}°", 
    #    transform=ax.transAxes, fontsize=10, color='blue', ha='left')

    #------軸ラベルとタイトル
    #plt.title("Static Simulation of Link Mechanism")
    plt.xlabel("X1 axis[m]")
    plt.ylabel("X2 axis[m]")
    plt.legend()
    plt.show()
    
else:
    print("到達可能範囲外です。")