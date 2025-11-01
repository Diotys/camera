"""
Classe CNC Controller Standalone - FluidNC
Compatible avec quality_control_main.py v3.0 FINAL
Toutes fonctionnalités : Frein Z, Limits, Jog, Télémètre, Auto-Z précis
Version: 1.1.0 FINAL

Configuration optimisée pour encodeur 25 microns (400 profils/10mm):
- Axe X: 512.30 steps/mm (51200 pulses/rev)
- Axes Y/Z: 256.15/343.626 steps/mm (25600 pulses/rev)
- Sortie encodeur: OUTA/OUTB vers Keyence LJ-X8000A
"""


import serial
import serial.tools.list_ports
import threading
import queue
import time
import re
from typing import Dict, Optional, Callable
from dataclasses import dataclass


@dataclass
class CNCStatus:
    """État du CNC"""
    position: Dict[str, float]
    work_position: Dict[str, float]
    machine_state: str
    feed_rate: float
    spindle_speed: float
    is_homed: bool
    brake_engaged: bool
    limit_x: bool
    limit_y: bool
    limit_z: bool


class CNCController:
    """Contrôleur CNC FluidNC avec frein Z, limits, télémètre"""
    
    def __init__(self, log_callback: Optional[Callable] = None):
        """
        Args:
            log_callback: Fonction pour logger (msg, level) où level = 'info'|'error'|'warning'|'sent'|'received'
        """
        self.log_callback = log_callback
        
        # Connexion CNC
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.is_connected = False

        # Thread safety - Lock pour l'état machine partagé
        self.state_lock = threading.Lock()

        # État machine (protégé par state_lock)
        self.position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.work_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.machine_state = "Idle"
        self.feed_rate = 0
        self.spindle_speed = 0
        self.is_homed = False

        # Frein Z (protégé par state_lock)
        self.brake_engaged = True
        self.brake_auto_release = True

        # Limit switches (protégé par state_lock)
        self.limit_x = False
        self.limit_y = False
        self.limit_z = False
        
        # Protection limites
        self.soft_limit_protection = False  # ⭐ Désactivé car pas de homing physique obligatoire
        self.workspace_limits = {"x": 450, "y": 450, "z": 50}
        
        # Télémètre laser
        self.telemetre_conn = None
        self.telemetre_lock = threading.Lock()
        self.telemetre_connected = False
        self.telemetre_distance = 0
        self.telemetre_running = False
        self.telemetre_thread = None
        
        # Communication
        self.tx_queue = queue.Queue()
        self.running = True
        
        # Threads
        self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.serial_thread.start()
        
        self.status_thread = threading.Thread(target=self._status_poller, daemon=True)
        self.status_thread.start()
    
    # ========== LOGGING ==========
    
    def _log(self, message: str, level: str = "info"):
        """Logger interne"""
        if self.log_callback:
            try:
                self.log_callback(message, level)
            except (TypeError, AttributeError) as e:
                # Log callback invalide - ne pas crasher pour autant
                print(f"Erreur log callback: {e}")
    
    # ========== CONNEXION CNC ==========
    
    @staticmethod
    def get_available_ports():
        """Retourne liste des ports COM disponibles"""
        return [port.device for port in serial.tools.list_ports.comports()]
    
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """Connecter au CNC"""
        try:
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=3.0,  # ⭐ Augmenté de 1.0 à 3.0
                write_timeout=3.0  # ⭐ Augmenté de 2.0 à 3.0
            )
            
            # ⭐ CRITIQUE: Flush AVANT d'attendre pour éliminer données boot
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            time.sleep(2.5)  # ⭐ Laisser FluidNC terminer le boot
            
            # ⭐ Flush à nouveau après le boot
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            self.is_connected = True
            self._log(f"✅ Connecté à {port}", "info")
            
            time.sleep(0.8)  # ⭐ Augmenté pour stabilité
            self.send_command("$I")
            time.sleep(0.5)
            
            # ⭐⭐⭐ CRITIQUE: Désactiver le homing obligatoire dans FluidNC
            self._log("🔧 Bypass homing obligatoire...", "info")
            self.send_command("$22=0")  # Désactive "homing required"
            time.sleep(0.3)
            self.send_command("$X")     # Unlock alarme
            time.sleep(0.5)
            
            # ⭐⭐⭐ Forcer mode relatif + unités métriques
            self.send_command("G91")    # Mode relatif
            time.sleep(0.2)
            self.send_command("G21")    # Unités mm
            time.sleep(0.2)
            
            # ⭐⭐ Auto-zero pour définir position actuelle = origine
            self._log("📍 Position actuelle = origine (0,0,0)...", "info")
            self.send_command("G92 X0 Y0 Z0")
            time.sleep(0.3)
            
            # ⭐⭐ Marquer comme homé
            self.is_homed = True
            self._log("✅ Machine prête - Homing bypass activé", "info")
            
            # Engagement frein
            self.engage_brake()
            
            return True
            
        except Exception as e:
            self._log(f"❌ Erreur connexion: {e}", "error")
            return False
    
    def disconnect(self):
        """Déconnecter le CNC"""
        if self.is_connected:
            self.engage_brake()
            time.sleep(0.2)

        self.is_connected = False
        if self.serial_conn:
            try:
                if self.serial_conn.is_open:
                    self.serial_conn.close()
            except (serial.SerialException, OSError) as e:
                # Port série déjà fermé ou erreur système
                self._log(f"Erreur fermeture port série: {e}", "warning")
            self.serial_conn = None

        self._log("Déconnecté", "info")

    def shutdown(self):
        """
        Arrêt propre de tous les threads et ressources
        À appeler avant de fermer l'application
        """
        self._log("🛑 Arrêt du contrôleur CNC...", "info")

        # Déconnecter le télémètre d'abord
        if self.telemetre_connected:
            self.telemetre_disconnect()

        # Signaler aux threads de s'arrêter
        self.running = False

        # Attendre que les threads se terminent proprement
        if hasattr(self, 'serial_thread') and self.serial_thread.is_alive():
            self._log("   Attente thread série...", "info")
            self.serial_thread.join(timeout=2.0)
            if self.serial_thread.is_alive():
                self._log("   ⚠️ Thread série ne répond pas (timeout)", "warning")

        if hasattr(self, 'status_thread') and self.status_thread.is_alive():
            self._log("   Attente thread status...", "info")
            self.status_thread.join(timeout=2.0)
            if self.status_thread.is_alive():
                self._log("   ⚠️ Thread status ne répond pas (timeout)", "warning")

        if hasattr(self, 'telemetre_thread') and self.telemetre_thread and self.telemetre_thread.is_alive():
            self._log("   Attente thread télémètre...", "info")
            self.telemetre_thread.join(timeout=2.0)
            if self.telemetre_thread.is_alive():
                self._log("   ⚠️ Thread télémètre ne répond pas (timeout)", "warning")

        # Déconnecter le CNC
        self.disconnect()

        self._log("✅ Contrôleur CNC arrêté proprement", "info")
    
    # ========== FREIN Z ==========
    
    def engage_brake(self):
        """Engager le frein Z"""
        if self.is_connected:
            self.send_command("M65 P0")
            self.brake_engaged = True
            self._log("🔒 FREIN ENGAGÉ (GPIO 32 LOW)", "info")
    
    def release_brake(self):
        """Libérer le frein Z"""
        if self.is_connected:
            self.send_command("M64 P0")
            self.brake_engaged = False
            self._log("🔓 FREIN LIBÉRÉ (GPIO 32 HIGH)", "info")
    
    # ========== MOUVEMENTS ==========
    
    def jog(self, axis: str, direction: int, distance: float, speed: int) -> bool:
        """
        Mouvement manuel (jog)
        
        Args:
            axis: 'X', 'Y', ou 'Z'
            direction: 1 (positif) ou -1 (négatif)
            distance: Distance en mm
            speed: Vitesse en mm/min
        """
        if not self.is_connected:
            self._log("❌ CNC non connecté", "error")
            return False
        
        # ⭐ AJOUT : Auto-unlock si Alarm
        if self.machine_state == "Alarm":
            self._log("🔓 Machine en alarme, envoi $X...", "warning")
            self.unlock_alarm()
            time.sleep(0.5)
        
        # ⭐ AJOUT : Vérifier si la machine est homée
        if not self.is_homed:
            self._log("⚠️ Machine non homée! Reconnectez le CNC.", "warning")
            return False
        
        move = distance * direction
        axis_upper = axis.upper()
        
        # DEBUG : Log avant le mouvement
        self._log(f"🎮 JOG {axis_upper}{move:+.3f} mm @ {speed} mm/min", "info")
        self._log(f"   État: {self.machine_state} | Homé: {self.is_homed}", "info")
        self._log(f"   Position actuelle: X={self.work_position['x']:.3f} Y={self.work_position['y']:.3f} Z={self.work_position['z']:.3f}", "info")
        
        # Protection limites
        if self.soft_limit_protection:
            current = self.work_position[axis.lower()]
            new_pos = current + move
            max_limit = self.workspace_limits[axis.lower()]
            
            if new_pos < -5 or new_pos > max_limit:
                self._log(f"⚠️ Limite {axis_upper} atteinte! ({new_pos:.1f} mm hors limites)", "warning")
                return False
        
        # Auto-libération frein Z
        if axis_upper == "Z" and self.brake_auto_release and self.brake_engaged:
            self._log("🔓 Libération frein pour mouvement Z...", "info")
            self.release_brake()
            time.sleep(0.3)
        
        # Commande mouvement relatif (G91 G1 au lieu de $J pour bypass homing)
        # ⭐⭐⭐ CRITIQUE: $J nécessite homing, G91 G1 fonctionne sans
        cmd = f"G91 G1 {axis_upper}{move:.3f} F{speed}"
        
        self._log(f"📤 Envoi: {cmd}", "sent")
        self.send_command(cmd)
        
        return True

    def move_relative(self, x=None, y=None, z=None, feed_rate=1000):
        """Déplacement relatif G91 avec vitesse contrôlée"""
        if not self.is_connected:
            self._log("❌ CNC non connecté", "error")
            return False

        # G91 = mode relatif (déjà défini à la connexion)
        # G1 = mouvement linéaire contrôlé (respecte feed_rate)
        parts = ["G91", "G1", f"F{feed_rate}"]

        if x is not None:
            parts.append(f"X{x:.3f}")
        if y is not None:
            parts.append(f"Y{y:.3f}")
        if z is not None:
            parts.append(f"Z{z:.3f}")

        cmd = " ".join(parts)
        self._log(f"📍 Déplacement relatif: {cmd}", "info")

        return self.send_command(cmd)
    
    def wait_idle(self, timeout=60):
        """
        Attendre que la machine soit Idle (thread-safe)

        Args:
            timeout: Timeout en secondes (défaut: 60s)

        Returns:
            True si machine Idle, False si timeout
        """
        if not self.is_connected:
            self._log("❌ CNC non connecté", "error")
            return False

        start = time.time()

        while time.time() - start < timeout:
            with self.state_lock:
                current_state = self.machine_state
            if current_state == "Idle":
                return True

            time.sleep(0.1)

        # Timeout
        with self.state_lock:
            final_state = self.machine_state
        self._log(f"⚠️ Timeout wait_idle après {timeout}s (état: {final_state})", "warning")
        return False


    
    def home_all(self):
        """Homing complet (tous les axes)"""
        if not self.is_connected:
            return False
        
        if self.brake_engaged:
            self.release_brake()
            time.sleep(0.2)
        
        # ⭐ CRITIQUE: Vider buffers avant commande critique
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            time.sleep(0.1)
        
        self.send_command("$H")
        self._log("🏠 Homing complet...", "info")
        
        def mark_homed():
            time.sleep(10)
            self.is_homed = True
            self._log("✅ Homing terminé", "info")
        
        threading.Thread(target=mark_homed, daemon=True).start()
        return True
    
    def zero_xy(self):
        """Zero XY personnalisé (homing X et Y uniquement)"""
        if not self.is_connected:
            return False
        
        if self.machine_state == "Alarm":
            self.unlock_alarm()
            time.sleep(0.5)
        
        self._log("🎯 Début Zero XY...", "info")
        
        if self.brake_engaged:
            self.release_brake()
            time.sleep(0.2)
        
        # ⭐ CRITIQUE: Vider buffers avant commande critique
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            time.sleep(0.1)
        
        self.send_command("$HX")
        time.sleep(0.5)
        self.send_command("$HY")
        
        def mark_homed():
            time.sleep(8)
            self.is_homed = True
            self.send_command("G92 X0 Y0")
            self._log("✅ Zero XY terminé!", "info")
        
        threading.Thread(target=mark_homed, daemon=True).start()
        return True

    def set_current_as_zero(self, axes: str = "XYZ") -> bool:
        """
        Définir la position actuelle comme étant 0 (sans mouvement physique)
        
        Args:
            axes: Axes à mettre à zéro ("XYZ", "XY", "Z", etc.)
        """
        if not self.is_connected:
            self._log("❌ CNC non connecté", "error")
            return False
        
        # Débloquer si en alarme
        if self.machine_state == "Alarm":
            self._log("🔓 Machine en alarme, envoi $X...", "warning")
            self.unlock_alarm()
            time.sleep(0.5)
        
        # Construire la commande G92
        cmd_parts = ["G92"]
        if "X" in axes.upper():
            cmd_parts.append("X0")
        if "Y" in axes.upper():
            cmd_parts.append("Y0")
        if "Z" in axes.upper():
            cmd_parts.append("Z0")
        
        cmd = " ".join(cmd_parts)
        
        self._log(f"📍 Définition position actuelle comme 0 pour {axes}", "info")
        self._log(f"   Avant: X={self.work_position['x']:.3f} Y={self.work_position['y']:.3f} Z={self.work_position['z']:.3f}", "info")
        
        self.send_command(cmd)
        time.sleep(0.3)
        
        # ⭐ IMPORTANT: Marquer comme "homé" pour que le jog fonctionne
        self.is_homed = True
        
        self._log(f"✅ Position 0 définie pour {axes} (machine considérée homée)", "info")
        return True
    
    def unlock_alarm(self):
        """Débloquer alarme"""
        if self.is_connected:
            self.send_command("$X")
            self._log("🔓 Unlock envoyé", "warning")
    
    def emergency_stop(self):
        """Arrêt d'urgence"""
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.reset_output_buffer()
            self.serial_conn.write(b'\x18')
            self._log("🛑 ARRÊT D'URGENCE", "error")
            time.sleep(0.3)
            self.engage_brake()
    
    def soft_reset(self):
        """Reset soft"""
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.serial_conn.write(b'\x18')
            time.sleep(0.5)
            self._log("🔄 Reset", "info")
            self.is_homed = False
            time.sleep(0.3)
            self.engage_brake()
    
    def feed_hold(self):
        """Pause"""
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(b'!')
            self._log("⏸ PAUSE", "warning")
    
    # ========== TÉLÉMÈTRE LASER ==========
    
    def connect_telemetre(self, port: str) -> bool:
        """Connecter le télémètre laser"""
        try:
            self.telemetre_conn = serial.Serial(
                port=port,
                baudrate=19200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                write_timeout=1
            )
            
            self.telemetre_conn.dtr = False
            self.telemetre_conn.rts = False
            time.sleep(0.3)
            
            self.telemetre_conn.reset_input_buffer()
            self.telemetre_conn.reset_output_buffer()
            
            # Auto-baudrate
            with self.telemetre_lock:
                self.telemetre_conn.write(bytes([0x55]))
            time.sleep(0.3)
            
            self.telemetre_connected = True
            self.telemetre_running = True
            
            # Thread de lecture
            self.telemetre_thread = threading.Thread(target=self._telemetre_worker, daemon=True)
            self.telemetre_thread.start()
            
            self._log(f"✅ Télémètre connecté sur {port}", "info")
            return True
            
        except Exception as e:
            self._log(f"❌ Erreur télémètre: {e}", "error")
            return False
    
    def disconnect_telemetre(self):
        """Déconnecter le télémètre"""
        self.telemetre_running = False
        
        if self.telemetre_conn and self.telemetre_conn.is_open:
            try:
                with self.telemetre_lock:
                    self.telemetre_conn.write(bytes([0x58]))  # Stop
                time.sleep(0.2)
                self.telemetre_conn.close()
            except (serial.SerialException, OSError) as e:
                self._log(f"Erreur déconnexion télémètre: {e}", "warning")
        
        self.telemetre_connected = False
        self._log("📡 Télémètre déconnecté", "info")
    
    def telemetre_measure(self):
        """Lancer une mesure télémètre"""
        if not self.telemetre_connected:
            return False
        
        cmd = [0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00]
        checksum = sum(cmd[1:]) & 0xFF
        cmd.append(checksum)
        
        with self.telemetre_lock:
            try:
                self.telemetre_conn.write(bytes(cmd))
                self._log("📡 Mesure lancée...", "info")
                return True
            except (serial.SerialException, OSError) as e:
                self._log(f"❌ Erreur écriture télémètre: {e}", "error")
                return False
    
    def telemetre_set_z_zero(self) -> bool:
        """Définir Z=0 à la distance mesurée"""
        if not self.is_connected or self.telemetre_distance == 0:
            return False
        
        self.send_command("G92 Z0")
        self._log(f"✅ Z=0 défini (distance: {self.telemetre_distance} mm)", "info")
        return True
    
    def auto_adjust_z_precise(self, target_distance: float, tolerance: float = 0.1) -> bool:
        """
        Ajustement automatique Z haute précision
        
        Args:
            target_distance: Distance cible en mm
            tolerance: Tolérance en mm (défaut 0.1mm)
        """
        if not self.is_connected or not self.telemetre_connected:
            self._log("❌ CNC et télémètre requis", "error")
            return False
        
        if self.brake_engaged:
            self.release_brake()
            time.sleep(0.3)
        
        self._log(f"🎯 AUTO Z PRÉCIS - Cible: {target_distance} mm", "info")
        
        # Thread pour ne pas bloquer
        threading.Thread(
            target=self._auto_z_worker,
            args=(target_distance, tolerance),
            daemon=True
        ).start()
        
        return True
    
    def _auto_z_worker(self, target_distance: float, tolerance: float):
        """Worker pour Auto Z précis"""
        max_iterations = 15
        
        for iteration in range(max_iterations):
            self._log(f"📬 Itération {iteration + 1}/{max_iterations}", "info")
            
            # Mesure
            old_dist = self.telemetre_distance
            self.telemetre_measure()
            
            # Attendre réponse (max 5s)
            for _ in range(50):
                time.sleep(0.1)
                if self.telemetre_distance != old_dist:
                    break
            
            current = self.telemetre_distance
            if current == 0:
                self._log("❌ Échec mesure", "error")
                return
            
            delta = current - target_distance
            self._log(f"📏 Distance: {current} mm | Delta: {delta:+.2f} mm", "info")
            
            # Vérifier tolérance
            if abs(delta) <= tolerance:
                self._log(f"✅ PRÉCISION ATTEINTE! ({current} mm)", "info")
                return
            
            # Mouvement adaptatif
            move = -delta
            
            if abs(delta) > 10:
                move = 20 if move > 0 else -20
                speed = 500
            elif abs(delta) > 2:
                speed = 300
            elif abs(delta) > 0.5:
                speed = 100
            else:
                speed = 50
            
            cmd = f"G91 G1 Z{move:.3f} F{speed}"
            self.send_command(cmd)
            
            # Attente mouvement
            wait = abs(move) / (speed / 60) + 0.5
            time.sleep(wait)
            
            # Attendre Idle
            for _ in range(50):
                if self.machine_state == "Idle":
                    break
                time.sleep(0.1)
            
            time.sleep(0.3)
        
        self._log(f"⚠️ Max itérations atteint", "warning")
    
    # ========== COMMUNICATION ==========
    
    def send_command(self, cmd: str):
        """Envoyer une commande G-code"""
        if self.is_connected and self.serial_conn:
            self.tx_queue.put(cmd)
    
    def get_status(self) -> CNCStatus:
        """Obtenir l'état actuel du CNC (thread-safe)"""
        with self.state_lock:
            return CNCStatus(
                position=self.position.copy(),
                work_position=self.work_position.copy(),
                machine_state=self.machine_state,
                feed_rate=self.feed_rate,
                spindle_speed=self.spindle_speed,
                is_homed=self.is_homed,
                brake_engaged=self.brake_engaged,
                limit_x=self.limit_x,
                limit_y=self.limit_y,
                limit_z=self.limit_z
            )
    
    def _serial_worker(self):
        """Thread de communication série CNC"""
        while self.running:
            try:
                # Envoi commandes
                if not self.tx_queue.empty() and self.is_connected and self.serial_conn:
                    cmd = self.tx_queue.get()
                    try:
                        if self.serial_conn and self.serial_conn.is_open:  # ⭐ Double vérification
                            self.serial_conn.reset_output_buffer()
                            self.serial_conn.write(f"{cmd}\n".encode())
                            self.serial_conn.flush()
                            self._log(f"→ {cmd}", "sent")
                    except serial.SerialTimeoutException:
                        self._log(f"⚠️ Timeout envoi: {cmd}", "warning")
                    except Exception as e:
                        self._log(f"Erreur envoi: {e}", "error")
                        self.is_connected = False  # ⭐ Marquer comme déconnecté
                
                # Réception - ⭐ PROTECTION RENFORCÉE
                if self.is_connected and self.serial_conn:
                    try:
                        # ⭐ Vérifier connexion AVANT chaque opération
                        if not self.serial_conn.is_open:
                            self._log("❌ Port série fermé", "error")
                            self.is_connected = False
                            continue
                        
                        if self.serial_conn.in_waiting > 0:
                            try:
                                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                                if line:
                                    self._parse_response(line)
                                    if not line.startswith('<'):
                                        self._log(f"← {line}", "received")
                            except (OSError, serial.SerialException) as e:
                                # ⭐ Erreurs I/O = perte connexion
                                self._log(f"❌ Connexion perdue: {e}", "error")
                                self.is_connected = False
                    except Exception as e:
                        if self.is_connected:
                            self._log(f"Erreur lecture: {e}", "error")
                            self.is_connected = False
                
                time.sleep(0.01)
            except Exception as e:
                self._log(f"Erreur worker: {e}", "error")
                time.sleep(0.1)
    
    def _parse_response(self, line: str):
        """Parser les réponses FluidNC (thread-safe)"""
        if line.startswith('<'):
            # Utiliser lock pour protéger toutes les écritures d'état
            with self.state_lock:
                # État machine
                match = re.search(r'<(\w+)', line)
                if match:
                    self.machine_state = match.group(1)

                # Position machine
                mpos = re.search(r'MPos:([-\d.]+),([-\d.]+),([-\d.]+)', line)
                if mpos:
                    self.position["x"] = float(mpos.group(1))
                    self.position["y"] = float(mpos.group(2))
                    self.position["z"] = float(mpos.group(3))

                # Position travail
                wpos = re.search(r'WPos:([-\d.]+),([-\d.]+),([-\d.]+)', line)
                if wpos:
                    self.work_position["x"] = float(wpos.group(1))
                    self.work_position["y"] = float(wpos.group(2))
                    self.work_position["z"] = float(wpos.group(3))

                # Vitesse
                fs = re.search(r'FS:([-\d.]+),([-\d.]+)', line)
                if fs:
                    self.feed_rate = float(fs.group(1))
                    self.spindle_speed = float(fs.group(2))

                # Limit switches
                pn = re.search(r'\|Pn:([XYZPDHRS]+)', line)
                if pn:
                    pins = pn.group(1)
                    self.limit_x = 'X' in pins
                    self.limit_y = 'Y' in pins
                    self.limit_z = 'Z' in pins
                else:
                    self.limit_x = False
                    self.limit_y = False
                    self.limit_z = False

        elif 'ALARM' in line.upper():
            self._log(f"⚠️ {line}", "warning")
    
        # ⭐ AJOUT : Capturer les erreurs
        elif line.startswith('error:'):
            self._log(f"❌ ERREUR CNC: {line}", "error")
    
        elif 'ok' in line.lower():
            # OK reçu, commande acceptée
            pass
    
    def _status_poller(self):
        """Thread pour polling status"""
        while self.running:
            if self.is_connected:
                self.send_command("?")
            time.sleep(0.5)
    
    def _telemetre_worker(self):
        """Thread de lecture télémètre"""
        buffer = bytearray()
        
        while self.telemetre_running:
            try:
                with self.telemetre_lock:
                    if self.telemetre_conn and self.telemetre_conn.is_open:
                        if self.telemetre_conn.in_waiting > 0:
                            data = self.telemetre_conn.read(self.telemetre_conn.in_waiting)
                            buffer.extend(data)
                
                # Parser
                while len(buffer) >= 12:
                    if buffer[0] == 0xAA:
                        if len(buffer) >= 12:
                            frame = bytes(buffer[:12])
                            self._parse_telemetre(frame)
                            buffer = buffer[12:]
                        else:
                            break
                    else:
                        try:
                            next_aa = buffer.index(0xAA)
                            buffer = buffer[next_aa:]
                        except ValueError:
                            buffer.clear()
                            break

                time.sleep(0.05)
            except (serial.SerialException, OSError, IndexError, ValueError) as e:
                # Erreur lecture série ou parsing - réessayer après délai
                self._log(f"Erreur lecture télémètre: {e}", "warning")
                time.sleep(0.1)
    
    def _parse_telemetre(self, data: bytes):
        """Parser réponse télémètre"""
        if len(data) < 12 or data[0] != 0xAA:
            return
        
        if data[3] == 0x22:  # Résultat mesure
            distance = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9]
            self.telemetre_distance = distance
            self._log(f"✅ Distance: {distance} mm", "info")
    
    # ========== CLEANUP ==========
    
    def shutdown(self):
        """Arrêt propre"""
        self.running = False
        if self.is_connected:
            self.engage_brake()
            time.sleep(0.2)
            self.disconnect()
        
        if self.telemetre_connected:
            self.disconnect_telemetre()
