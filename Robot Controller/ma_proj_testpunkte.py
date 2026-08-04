'''
Programm-Funktion: Programm gibt es genauso im Roboter-Touchpad. Es wurden 4 Positionen (Home, GP1, Testpunt1, Testpunkt2) geteacht die der Roboter
ohne Kollision gut erreichen kann. Es sollen hier die Positionen wie nachfolgend abgefahren werden:
- MoveJoint auf Home
- MoveJoint auf GP1
- MoveJoint auf Testpunkt1
- MoveLinear auf Testpunkt2
- MoveLinear auf Testpunkt1
- MoveJoint auf Home

Positionen der Punkte:
PUNKT: 'Home'
  -> Joints (rad)   : [0, 0, 1.5708001839725434, 0, 1.5708001839725434, 0]
  -> Kartesisch (m) : [0.41999872, 0, 0.43449838, 3.14159265, -7.71e-06, 3.14159265]

PUNKT: 'GP1'
-> Joints (rad)   : [0.49755461022518865, 0.4815814025134143, 1.85529815866986, -1.0062059410277526e-05, 0.8047602550602729, 0.4975593406967544]
  -> Kartesisch (m) : [0.42062980282766194, 0.22845593505593, 0.10008658351921855, -3.1415637712839826, -3.7983070090828824e-05, -3.141590408721063]
PUNKT: 'Testpunkt1'
-> Joints (rad)   : [0.2776315620398495, 0.46921546587748664, 1.5517314499641786, -1.942170455001727e-05, 1.1206901001681349, 0.27763792332699533]
  -> Kartesisch (m) : [0.5289242764139978, 0.1507360655889368, 0.2106883876503932, -3.141563677481791, -3.7871120525438625e-05, -3.141590565092438]

PUNKT: 'Testpunkt2'
  -> Joints (rad)   : [0.2776313891512311, 0.6642829096102066, 1.6266402349740268, -2.307859679586962e-05, 0.85071393371505, 0.2776443637461929]
  -> Kartesisch (m) : [0.5289243410130823, 0.15073600938675372, 0.07671484252800718, -3.1415637930947407, -3.796823753012352e-05, -3.1415904089190376]
'''
import time
from neurapy.robot import Robot

def load_all_points(r):
    """Liest alle benötigten Punkte in der passenden Repräsentation aus der Datenbank."""
    print("Lade Punkte aus der Roboter-Datenbank...")
    
    p = {
        # Für MoveJoint -> Joint-Repräsentation (Achs-Winkel von A1 bis A6 in Grad)
        "home_joint": r.get_point("Home", representation="Joint"),
        "gp1_joint": r.get_point("GP1", representation="Joint"),
        "tp1_joint": r.get_point("Testpunkt1", representation="Joint"),
        
        # Für MoveLinear -> Cartesian-Repräsentation (X, Y, Z, Rx, Ry, Rz) in m
        "tp1_cart": r.get_point("Testpunkt1", representation="Cartesian"),
        "tp2_cart": r.get_point("Testpunkt2", representation="Cartesian"),
    }
    
    print("Alle Punkte erfolgreich geladen.\n")
    return p

def compute_inverse_kinematics(r, pose_cartesian):
    target_cartesian = pose_cartesian
    print("Übergebene target_cartesian: ", target_cartesian)
    target_joints_pos = r.compute_inverse_kinematics(
        target_pose=target_cartesian,
        reference_joint=r.get_current_joint_angles()
    )
    print("Berechnete target_joints: ", target_joints_pos)
    return target_joints_pos

    
def move(r, point_data, motion_type, motion_speed):
    """Führt eine Bewegung (Joint oder Linear) für einen übergebenen Punkt aus."""
    if motion_type == "joint":
        print(f"Joint-Bewegung (Punkt: {point_data}, Speed: {motion_speed} %)...\n")
        r.move_joint(
            target_joint=[point_data],
            speed=motion_speed,
            current_joint_angles=r.get_current_joint_angles()
        )

    elif motion_type == "linear":
        print(f"Lineare Bewegung (Punkt: {point_data}, Speed: {motion_speed} m/s)...\n")
        r.move_linear(
            target_pose=[point_data],
            speed=motion_speed,
            current_joint_angles=r.get_current_joint_angles()
        )
        

def main():
    #Verbindung und Initialisierung des Roboters
    print("Roboterverbindung aufbauen...")
    r = Robot()
    r.init_program()
    r.power_on()
    print("Verbindung steht!\n")

    # Automatik-Modus sicherstellen
    if hasattr(r, 'is_robot_in_teach_mode') and r.is_robot_in_teach_mode():
        print("Schalte Roboter in Automatik-Modus...")
        r.switch_to_automatic_mode()
        time.sleep(1.0)

    # Globaler Geschwindigkeits-Override auf 20% Drosseln
    r.set_override(0.2)

    #Punkte aus der Datenbank auslesen
    pts = load_all_points(r)

    #Bewegungsablauf ausführen
    print("--- Beginn des Fahrprogramms")

    # MoveJoint auf Home (20% Achsgeschwindigkeit)
    move(r, pts["home_joint"], "joint", 50.0)

    # MoveJoint auf GP1
    move(r, pts["gp1_joint"], "joint", 50.0)
    #target_joints_gp1 = compute_inverse_kinematics(r, pts['gp1_cart'])
    #move(r, target_joints_gp1, "joint", 50.0)

    # MoveJoint auf Testpunkt1
    target_joints_tp1 = compute_inverse_kinematics(r, pts['tp1_cart'])
    move(r, target_joints_tp1, "joint", 50.0)


    # MoveLinear auf Testpunkt2 (0.1 m/s = 10 cm/s)
    move(r, pts["tp2_cart"], "linear", 0.2)

    # MoveLinear auf Testpunkt1
    move(r, pts["tp1_cart"], "linear", 0.2)

    # MoveJoint zurück auf Home
    move(r, pts["home_joint"], "joint", 50.0)

    # 4. Programm sauber beenden
    print("\n -- Ablauf beendet")
    r.stop()

if __name__ == "__main__":
    main()