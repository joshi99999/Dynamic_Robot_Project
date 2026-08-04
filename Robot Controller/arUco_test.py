import math
import time
from neurapy.robot import Robot

def load_all_points(r):
    """Liest alle benötigten Punkte in der passenden Repräsentation aus der Datenbank."""
    print("Lade Punkte aus der Roboter-Datenbank...")
    
    p = {
        # Für MoveJoint -> Joint-Repräsentation (Achs-Winkel von A1 bis A6 in Grad)
        "home_joint": r.get_point("Home", representation="Joint"),
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
    # 1. Verbindung aufbauen und initialisieren
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

    # Globaler Geschwindigkeits-Override auf 20% Drosseln (Sicherheit!)
    r.set_override(0.2)

    #Punkte aus der Datenbank auslesen
    pts = load_all_points(r)

    #Bewegungsablauf ausführen
    print("--- Beginn des Fahrprogramms")

    # MoveJoint auf Home (20% Achsgeschwindigkeit)
    move(r, pts["home_joint"], "joint", 75.0)
    
    

    #Vorpos in Joints
    #Punkt in der Datenbank löschen, falls er schon existiert --> sonst kann er nicht neu erstellt werden und es 
    # kommt zu einem Fehler
    try:
        r.delete_pose_in_DB("Vorpos_cartesian")
    except Exception:
        pass
    #Punkt erstellen im Koordinatensystem des ArUco-Markers --> wichtig: die Pose muss in der Form 
    # [x, y, z, a, b, c] übergeben werden (in Metern und Radiant)
    #a und c müssen auf math.pi (180°) gesetzt werden damit der Roboter ohne Drehung verfährt
    r.create_point(name='Vorpos_cartesian', reference_frame_name='World', 
                   target_end_effector_pose=[float(0.225), float(-0.31), float(0.2), math.pi, float(0.0), math.pi])
    cartesian_pose_world = r.get_point('Vorpos_cartesian', representation='Cartesian')
    print(f"Vorpos_cartesian (World): {cartesian_pose_world}")
    target_joints_Vorpos = r.compute_inverse_kinematics(target_pose=cartesian_pose_world, 
                                                        reference_joint=r.get_current_joint_angles())
    print(f"target_joints_Vorpos: {target_joints_Vorpos}")
    move(r, target_joints_Vorpos, "joint", 75.0)


    #Hauptpos_cartesian = [0.1, 0.2, 0.0, -3.141563677481791, -3.7871120525438625e-05, -3.141590565092438]
    try:
        r.delete_pose_in_DB("Hauptpos_cartesian")
    except Exception:
        pass
    r.create_point(name='Hauptpos_cartesian',reference_frame_name='World', 
                   target_end_effector_pose=[float(0.225), float(-0.31), float(0.05), math.pi, float(0.0), math.pi])
    move(r, r.get_point("Hauptpos_cartesian", representation="Cartesian"), "linear", 0.2)

    move(r, r.get_point("Vorpos_cartesian", representation="Cartesian"), "linear", 0.2)
    move(r, pts["home_joint"], "joint", 75.0)
    

    print("\n -- Ablauf beendet")
    r.stop()

if __name__ == "__main__":
    main()