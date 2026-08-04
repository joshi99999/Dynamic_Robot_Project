import time
from neurapy.robot import Robot
import math

def main():
    print("Roboterverbindung aufbauen ...")
    r = Robot()
    r.init_program()
    r.power_on()
    print("Verbindunden\n")

    try:
        point_names = r.get_point_names()
        print(f"Gefundene Punkte in der Datenbank: {point_names}\n")
    except Exception as e:
        print(f"Fehler beim Auslesen der Punkteliste: {e}")
        point_names = []

    # 3. Details für jeden Punkt auslesen und ausgeben
    if point_names:
        for name in point_names:
            print(f"PUNKT: '{name}'")
            
            # Gelenkwinkel (Joints in Radian) auslesen
            try:
                joint_vals = r.get_point(name, representation="Joint")
                print(f"  -> Joints (rad)   : {joint_vals}")
            except Exception as e:
                print(f"  -> Joints Fehler  : {e}")

            # Kartesische Position [X, Y, Z (m), Rx, Ry, Rz (rad)] auslesen
            try:
                cart_vals = r.get_point(name, representation="Cartesian")
                print(f"  -> Kartesisch (m) : {cart_vals}")
            except Exception as e:
                print(f"  -> Kart. Fehler   : {e}\n")
    else:
        print("Keine Punkte gefunden")

    # 4. Aktuelle Position des Roboters ausgeben (Ist-Zustand)
    print("\nAKTUELLE ROBOTER-POSITION:")
    try:
        curr_joints = r.get_current_joint_angles()
        print(f"  -> Aktuelle Joints: {curr_joints}")
    except Exception as e:
        print(f"  -> Fehler beim Auslesen der aktuellen Joints: {e}")

    #Funktion um die Frame auszugeben
    frame = r.get_reference_frame("World")
    print("frame: ", frame)
    #Output: frame:  [0, 0, 0, 0, 0, 0]

    frame_ArUco = r.get_reference_frame("ArUco")
    print("frame ArUco: ", frame_ArUco)

    x_offset = 0.02 # in meters
    y_offset = 0.1 # in meters
    z_offset = 0.03 # in meters
    a_offset = 10 * math.pi / 180 # Offset in radiand angeben, aber die Bildverarbeitung liefert die Werte in Grad
    b_offset = 15 * math.pi / 180 # Offset in radiand angeben, aber die Bildverarbeitung liefert die Werte in Gradd 
    c_offset = 20 * math.pi / 180 # Offset in radiand angeben, aber die Bildverarbeitung liefert die Werte in Grad
    frame_ArUco_new = r.get_reference_frame_with_offset("ArUco",[x_offset,y_offset,z_offset, a_offset,b_offset,c_offset])
    print("frame_ArUco_new: ", frame_ArUco_new)
    #Output:
    #Wir wollen folgendes damit umsetzten: Wir haben ein Frame ArUco was ein Abholbereich kennzeichnte. 
    # Wir legen in das Frame an einer Stelle (bspw. unten links) den Ursprung des Frames fest, was bei uns 0.42, 0.23, 0.1 in m
    # ist. Wenn der X:0, Y:0, Z:0
    #frame:  [0, 0, 0, 0, 0, 0]
    #frame ArUco:  [0.42, 0.23, 0.1, 3.14159, 0, 3.14159]
    #frame_ArUco_new:  [0.400000265359261, 0.33000013267878553, 0.07000026535908496, 3.14159, 0, 3.14159]

    r.stop()

if __name__ == "__main__":
    main()

'''
Gefundene Punkte in der Datenbank: ['Parking', 'Home', 'Rotate', 'SafeRotateIn', 'Grip', 'GP1', 'GripSafe', 'RotateGripSafe', 'GripRotate', 'GP2', 'GP3', 'GP4', 'GP5', 'GP6', 'GP7', 'GP8', 'GP9', 'GP10', 'GP11', 'GP12', 'GP13', 'GP14', 'GP15', 'GP16', 'Testpunkt1', 'RotateGripSafe_1', 'Testpunkt2']

PUNKT: 'Parking'
  -> Joints (rad)   : [1.5708001839725434, 0, -2.49582083, 0, 0, 0]
  -> Kartesisch (m) : [1.36e-06, -0.3523627, 0.13239891, 3.14159265, -0.64577182, -1.57079247]
PUNKT: 'Home'
  -> Joints (rad)   : [0, 0, 1.5708001839725434, 0, 1.5708001839725434, 0]
  -> Kartesisch (m) : [0.41999872, 0, 0.43449838, 3.14159265, -7.71e-06, 3.14159265]
PUNKT: 'Rotate'
  -> Joints (rad)   : [0.47705067493949777, 0.60515512277826, 2.1488419346390137, 1.3768666771924802, 1.649384241594796, -0.006646358729038641]
  -> Kartesisch (m) : [0.09517146416925912, 0.462679671648781, 0.14369837554431952, 1.570614645299464, -1.1841953155275762, -2.455311236992754]
PUNKT: 'SafeRotateIn'
  -> Joints (rad)   : [0.4770506715506581, 0.7966107850767938, 2.0966609835896803, 1.3679860548737737, 1.6218933505830713, -0.1434734201873638]
  -> Kartesisch (m) : [0.09517149628746432, 0.46267970679260834, 0.07857922366123646, 1.5706166855379649, -1.1841953643975522, -2.4553133935449156]
PUNKT: 'Grip'
  -> Joints (rad)   : [1.1549863576670487, 0.7732375261190063, 2.210329980089926, -0.961660872361445, 1.653864255686792, -0.5019982175095095]
  -> Kartesisch (m) : [0.32727040291981213, -0.018612758578755786, 0.0742560514926399, 1.5921957760520487, -1.1981499305765437, 0.5210375191881141]
PUNKT: 'GP1'
  -> Joints (rad)   : [-0.2570242578744364, 1.426342856284765, 1.5653190870815907, 0.00013951001150341256, -2.9916171905448734, 0.25715730355210165]
  -> Kartesisch (m) : [0.42437019307502527, -0.11154406304306366, 0.024913412383471216, 3.153589560589923e-05, 3.7983083089194656e-05, -4.898357732038735e-06]
PUNKT: 'GripSafe'
  -> Joints (rad)   : [1.1472570788944527, 0.19814198482163636, 2.274892023813504, -1.0572215252439878, 1.9315416407831256, -0.9415668195911141]
  -> Kartesisch (m) : [0.327275, -0.018613, 0.26, 1.5927009636566893, -1.1994754999751074, 0.5087037702243767]
PUNKT: 'RotateGripSafe'
  -> Joints (rad)   : [1.1472438157773677, 0.19815528492706286, 2.2748747934625375, -1.057216329075547, 1.9315473756856427, 2.1932056106901867]
  -> Kartesisch (m) : [0.327275, -0.018613, 0.26, -1.5923246972537548, 1.1926581946979655, -2.632504958736166]
PUNKT: 'GripRotate'
  -> Joints (rad)   : [1.1472438233295392, 0.7548199935310338, 2.2077519315412646, -0.9590715293076756, 1.6663116989082978, 2.617326744407196]
  -> Kartesisch (m) : [0.3272748510579619, -0.018613236187863494, 0.08051792380870687, -1.5923303316245723, 1.1927521489897628, -2.6325111002236854]
PUNKT: 'GP2'
  -> Joints (rad)   : [0.964061235, 0.710267559, 2.350145453, 1.921409432, 1.509025887, -0.289715641]
  -> Kartesisch (m) : [-0.054232093, 0.539173501, 0.076829333, 1.664818897, -1.203324548, -2.619566398]
PUNKT: 'GP3'
  -> Joints (rad)   : [1.234157202, 0.70856168, 2.282005022, 2.250368541, 1.439117034, -0.282010908]
  -> Kartesisch (m) : [-0.091898187, 0.613989448, 0.079770074, 1.6633510739999995, -1.16987275, -2.6821159989999996]
PUNKT: 'GP4'
  -> Joints (rad)   : [1.234139226, 0.577490961, 2.316296758, 2.237997451, 1.378796685, -0.205405121]
  -> Kartesisch (m) : [-0.091898195, 0.613989329, 0.117675126, 1.6633504839999995, -1.169872648, -2.6821156359999994]
PUNKT: 'GP5'
  -> Joints (rad)   : [1.468628188, 0.563997098, 2.358123201, 2.478064695, 1.360444717, -0.264816916]
  -> Kartesisch (m) : [-0.164411898, 0.613989271, 0.117675104, 1.6633514600000001, -1.1698727559999997, -2.6821160180000003]
PUNKT: 'GP6'
  -> Joints (rad)   : [1.468628188, 0.674916343, 2.33018685, 2.487206037, 1.426082317, -0.316461809]
  -> Kartesisch (m) : [-0.164411926, 0.613989453, 0.087047248, 1.6633511999999993, -1.169872728, -2.6821160999999996]
PUNKT: 'GP7'
  -> Joints (rad)   : [1.220615403, 0.658314446, 2.478230976, 2.246532091, 1.531569337, -0.395456951]
  -> Kartesisch (m) : [-0.113970543, 0.541308943, 0.087047243, 1.663351108, -1.169872725, -2.682116218]
PUNKT: 'GP8'
  -> Joints (rad)   : [0.785686534, 0.688555889, 2.268038157, 1.801393037, 1.491353649, -0.219164135]
  -> Kartesisch (m) : [0.030861963, 0.54639207, 0.087047232, 1.6633511750000003, -1.169872668, -2.682116204]
PUNKT: 'GP9'
  -> Joints (rad)   : [0.329363202, 0.688143557, 2.269684116, 1.352505543, 1.574375865, -0.219917268]
  -> Kartesisch (m) : [0.106541011, 0.423832988, 0.087047248, 1.6633510309999995, -1.169872731, -2.6821159609999996]
PUNKT: 'GP10'
  -> Joints (rad)   : [0.529101657, 0.697741797, 2.232160009, 1.547946529, 1.538841515, -0.18750743]
  -> Kartesisch (m) : [0.09251499, 0.488741934, 0.087047238, 1.6633511439999997, -1.169872691, -2.682116135]
PUNKT: 'GP11'
  -> Joints (rad)   : [0.927342694, 0.658341036, 2.496022681, 1.953793055, 1.539575548, -0.411240176]
  -> Kartesisch (m) : [-0.058224023, 0.502641335, 0.087047209, 1.663351409, -1.169872668, -2.682116078]
PUNKT: 'GP12'
  -> Joints (rad)   : [1.439988137, 0.661525844, 2.420108609, 2.463787738, 1.487970727, -0.361793264]
  -> Kartesisch (m) : [-0.160712113, 0.577353648, 0.087047304, 1.6633506670000004, -1.169872819, -2.6821155310000004]
PUNKT: 'GP13'
  -> Joints (rad)   : [1.439943946, 0.52408551, 2.453792145, 2.455685278, 1.407398536, -0.29595343]
  -> Kartesisch (m) : [-0.160712049, 0.577353598, 0.121070118, 1.6633511640000003, -1.169872727, -2.682116065]
PUNKT: 'GP14'
  -> Joints (rad)   : [1.2686030241686377, 0.5055275246147845, 2.317396875516199, 2.2500609347688574, 1.2461213546012695, -0.14684868987049585]
  -> Kartesisch (m) : [-0.09605395654763177, 0.6222374443760962, 0.10990648489544502, 1.86655777459131, -1.1599794596337403, -2.8794932312951595]
PUNKT: 'GP15'
  -> Joints (rad)   : [1.2680698233551577, 0.5917736219415701, 2.3007220239876722, 2.2663918806704775, 1.291571423215565, -0.20317726132919714]
  -> Kartesisch (m) : [-0.09599352782269455, 0.6221171002459942, 0.085073735546012, 1.8636895633370085, -1.1602342271425576, -2.8767086937345847]
PUNKT: 'GP16'
  -> Joints (rad)   : [-0.072965204, -0.116972776, 2.184996467, -0.035490908, 0.958484077, -0.000117595]
  -> Kartesisch (m) : [0.342547141, -0.029855958, 0.232720068, -3.110182925, 0.114702283, 3.089297991]
PUNKT: 'Testpunkt1'
  -> Joints (rad)   : [-0.5395268416860305, 1.7061227160552443, 1.454818775206177, -0.0004156929999277808, 3.1222925718939325, 0.5391064804366889]
  -> Kartesisch (m) : [0.3160755132524184, -0.18926335164858127, -0.0856885979832038, 3.162969779725619e-05, 3.787113310885935e-05, -4.741986654200502e-06]
PUNKT: 'RotateGripSafe_1'
  -> Joints (rad)   : [1.1472438157773677, 0.19815528492706286, 2.2748747934625375, -1.057216329075547, 1.9315473756856427, 2.1932056106901867]
  -> Kartesisch (m) : [0.327275, -0.018613, 0.26, -1.5923246972537548, 1.1926581946979655, -2.632504958736166]
PUNKT: 'Testpunkt2'
  -> Joints (rad)   : [-0.5395281111391433, 1.3511997132259477, 1.796284672677324, -0.001298685882662333, 3.135749681853134, 0.5382245486878068]
  -> Kartesisch (m) : [0.31607544865318477, -0.1892637633614248, 0.04828494713856136, 3.151408484781224e-05, 3.796825052796405e-05, -4.898159796757327e-06]

AKTUELLE ROBOTER-POSITION:
  -> Aktuelle Joints: [-0.15124054379857163, 0.20526318262974932, 1.3465463868447796, -3.370563254631536e-05, 1.5898639776333754, -0.15125514957267505]
frame:  [0, 0, 0, 0, 0, 0]
frame ArUco:  [0.425, -0.11, 0.025, 0, 0, 0]
frame_ArUco_new:  [0.445, -0.009999999999999995, 0.055, 0, 0, 0]
'''