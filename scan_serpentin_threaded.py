"""
CORRECTIONS THREADING POUR SCAN SERPENTIN
==========================================

🎯 Résout le problème de GUI figée pendant la capture

À ajouter dans quality_control_main.py
"""

from PyQt6.QtCore import QThread, pyqtSignal
import time


class ScanSerpentinThread(QThread):
    """Thread pour exécuter le scan serpentin sans bloquer la GUI"""
    
    # Signaux pour communiquer avec le GUI
    log_signal = pyqtSignal(str)  # Pour les messages de log
    progress_signal = pyqtSignal(int, int)  # (profils_reçus, total_attendu)
    finished_signal = pyqtSignal(bool, str)  # (succès, message)
    profile_received_signal = pyqtSignal(int)  # Nombre de profils
    
    def __init__(self, keyence_controller, cnc_controller, params):
        super().__init__()
        self.keyence = keyence_controller
        self.cnc = cnc_controller
        self.params = params
        self.is_cancelled = False
        
    def cancel(self):
        """Annuler le scan en cours"""
        self.is_cancelled = True
        self.log_signal.emit("🛑 Annulation demandée...")
        
    def run(self):
        """Exécution du scan serpentin dans le thread"""
        try:
            self.log_signal.emit("=" * 60)
            self.log_signal.emit("🐍 DÉMARRAGE SCAN SERPENTIN")
            self.log_signal.emit("=" * 60)
            
            # Paramètres
            longueur = self.params['longueur']
            largeur = self.params['largeur']
            vitesse = self.params['vitesse']
            pas_y = self.params['pas_y']
            
            self.log_signal.emit(f"\n📐 PARAMÈTRES:")
            self.log_signal.emit(f"   Longueur (X): {longueur} mm")
            self.log_signal.emit(f"   Largeur (Y): {largeur} mm")
            self.log_signal.emit(f"   Vitesse: {vitesse} mm/min")
            self.log_signal.emit(f"   Pas Y: {pas_y} mm")
            
            # Calculs
            nb_passes = int(largeur / pas_y)
            profils_par_passe = int(longueur / 0.025)  # 0.025mm = résolution encodeur (400 profils/10mm)
            profils_total = profils_par_passe * nb_passes
            
            self.log_signal.emit(f"\n📊 CALCULS:")
            self.log_signal.emit(f"   Nombre de passes: {nb_passes}")
            self.log_signal.emit(f"   Profils par passe: {profils_par_passe}")
            self.log_signal.emit(f"   Profils total attendu: {profils_total}")
            
            # Démarrer la capture encodeur
            self.log_signal.emit("\n🎯 Configuration capture encodeur...")
            if not self.keyence.start_capture_encoder():
                self.finished_signal.emit(False, "❌ Échec démarrage capture")
                return
            
            self.log_signal.emit("✅ Capture encodeur démarrée")
            
            # Attendre un peu que le Keyence soit prêt
            time.sleep(0.5)
            
            # Compteur de profils
            profils_recus = 0
            
            # Boucle serpentin
            for passe in range(nb_passes):
                if self.is_cancelled:
                    self.log_signal.emit("🛑 Scan annulé par l'utilisateur")
                    break
                
                direction = 1 if passe % 2 == 0 else -1
                self.log_signal.emit(f"\n{'='*60}")
                self.log_signal.emit(f"🔄 PASSE {passe + 1}/{nb_passes} - Direction: {'→' if direction > 0 else '←'}")
                self.log_signal.emit(f"{'='*60}")
                
                # Déplacement en X
                distance_x = longueur * direction
                temps_deplacement = (abs(distance_x) / vitesse) * 60  # en secondes
                
                self.log_signal.emit(f"📍 Déplacement relatif: X{distance_x:+.3f} mm à {vitesse} mm/min")
                self.log_signal.emit(f"⏱️  Temps estimé: {temps_deplacement:.1f}s")
                
                # Envoyer commande CNC
                self.cnc.move_relative(x=distance_x, feed_rate=vitesse)
                self.log_signal.emit("   → Commande CNC envoyée")
                
                # Attendre que le CNC commence à bouger
                time.sleep(0.5)
                
                # Attendre les profils encodeur avec timeout
                self.log_signal.emit(f"⏳ Attente profils encodeur... (timeout: {int(temps_deplacement + 60)}s)")
                
                start_time = time.time()
                timeout = temps_deplacement + 60
                profils_avant = profils_recus
                
                # Boucle d'attente non-bloquante
                while True:
                    if self.is_cancelled:
                        break
                        
                    # Vérifier les profils reçus
                    profils_dispo = self.keyence.get_profile_count()
                    
                    if profils_dispo > profils_recus:
                        nouveaux = profils_dispo - profils_recus
                        profils_recus = profils_dispo
                        self.log_signal.emit(f"   📦 +{nouveaux} profils (total: {profils_recus}/{profils_total})")
                        self.profile_received_signal.emit(profils_recus)
                        self.progress_signal.emit(profils_recus, profils_total)
                    
                    # Vérifier si on a reçu assez de profils pour cette passe
                    profils_passe = profils_recus - profils_avant
                    if profils_passe >= profils_par_passe * 0.95:  # 95% = OK
                        self.log_signal.emit(f"✅ Passe complète: {profils_passe} profils reçus")
                        break
                    
                    # Timeout ?
                    elapsed = time.time() - start_time
                    if elapsed > timeout:
                        self.log_signal.emit(f"⚠️  TIMEOUT après {elapsed:.1f}s")
                        self.log_signal.emit(f"   Profils reçus cette passe: {profils_passe}/{profils_par_passe}")
                        break
                    
                    # Petite pause pour ne pas surcharger le CPU
                    time.sleep(0.1)
                
                # Attendre que le CNC soit immobile
                self.log_signal.emit("⏸️  Attente CNC idle...")
                try:
                    self.cnc.wait_idle(timeout=int(temps_deplacement + 60))
                except Exception as e:
                    self.log_signal.emit(f"⚠️  Timeout wait_idle: {e}")
                    time.sleep(5)  # Fallback
                
                # Si pas dernière passe, déplacer en Y
                if passe < nb_passes - 1:
                    self.log_signal.emit(f"\n🔽 Déplacement Y: +{pas_y} mm")
                    self.cnc.move_relative(y=pas_y, feed_rate=vitesse)
                    time.sleep(2)  # Attente stabilisation
            
            # Arrêter la capture
            self.log_signal.emit("\n🛑 Arrêt capture...")
            self.keyence.stop_capture()
            
            # Résumé
            self.log_signal.emit("\n" + "=" * 60)
            self.log_signal.emit("📊 RÉSUMÉ CAPTURE")
            self.log_signal.emit("=" * 60)
            self.log_signal.emit(f"✅ Profils reçus: {profils_recus}")
            self.log_signal.emit(f"📏 Profils attendus: {profils_total}")
            self.log_signal.emit(f"📈 Taux réussite: {profils_recus/profils_total*100:.1f}%")
            
            # Distance réelle parcourue
            distance_reelle = profils_recus * 0.025
            distance_attendue = longueur * nb_passes
            self.log_signal.emit(f"\n🎯 Distance réelle: {distance_reelle:.2f} mm")
            self.log_signal.emit(f"📐 Distance attendue: {distance_attendue:.2f} mm")
            
            if profils_recus >= profils_total * 0.9:
                self.finished_signal.emit(True, f"✅ Scan réussi: {profils_recus} profils")
            else:
                self.finished_signal.emit(False, f"⚠️ Scan incomplet: {profils_recus}/{profils_total} profils")
            
        except Exception as e:
            self.log_signal.emit(f"\n❌ ERREUR: {e}")
            import traceback
            self.log_signal.emit(traceback.format_exc())
            self.finished_signal.emit(False, f"❌ Erreur: {e}")


# ============================================================================
# MODIFICATIONS À FAIRE DANS LA CLASSE PRINCIPALE
# ============================================================================

class QualityControlApp_MODIFICATIONS:
    """
    Voici les modifications à intégrer dans votre QualityControlApp
    """
    
    def __init__(self):
        # ... votre code existant ...
        
        # Ajouter ces attributs
        self.scan_thread = None
        self.scan_running = False
    
    def setup_serpentin_tab(self):
        """
        REMPLACER la méthode existante par celle-ci
        """
        # ... votre layout existant ...
        
        # MODIFIER les boutons
        self.btn_start_serpentin = QPushButton("▶️ Démarrer Scan Serpentin")
        self.btn_start_serpentin.clicked.connect(self.start_scan_serpentin_threaded)
        
        self.btn_cancel_serpentin = QPushButton("🛑 Annuler Scan")
        self.btn_cancel_serpentin.clicked.connect(self.cancel_scan_serpentin)
        self.btn_cancel_serpentin.setEnabled(False)
        
        # ... reste du layout ...
    
    def start_scan_serpentin_threaded(self):
        """
        NOUVELLE méthode threadée pour lancer le scan
        """
        if self.scan_running:
            self.log("⚠️ Un scan est déjà en cours!")
            return
        
        # Récupérer les paramètres
        params = {
            'longueur': self.spin_serpentin_longueur.value(),
            'largeur': self.spin_serpentin_largeur.value(),
            'vitesse': self.spin_serpentin_vitesse.value(),
            'pas_y': self.spin_serpentin_pas_y.value()
        }
        
        # Créer et démarrer le thread
        self.scan_thread = ScanSerpentinThread(
            self.keyence_controller,
            self.cnc_controller,
            params
        )
        
        # Connecter les signaux
        self.scan_thread.log_signal.connect(self.log)
        self.scan_thread.progress_signal.connect(self.update_scan_progress)
        self.scan_thread.profile_received_signal.connect(self.update_profile_count)
        self.scan_thread.finished_signal.connect(self.scan_finished)
        
        # Démarrer
        self.scan_thread.start()
        self.scan_running = True
        
        # Modifier l'état des boutons
        self.btn_start_serpentin.setEnabled(False)
        self.btn_cancel_serpentin.setEnabled(True)
        
        self.log("🚀 Thread de scan démarré - GUI reste réactif!")
    
    def cancel_scan_serpentin(self):
        """
        NOUVELLE méthode pour annuler le scan
        """
        if self.scan_thread and self.scan_running:
            self.log("🛑 Demande d'annulation...")
            self.scan_thread.cancel()
    
    def update_scan_progress(self, current, total):
        """
        NOUVELLE méthode pour mettre à jour la barre de progression
        """
        if hasattr(self, 'progress_bar_serpentin'):
            percentage = int((current / total) * 100) if total > 0 else 0
            self.progress_bar_serpentin.setValue(percentage)
    
    def update_profile_count(self, count):
        """
        NOUVELLE méthode pour afficher le nombre de profils
        """
        if hasattr(self, 'lbl_profile_count'):
            self.lbl_profile_count.setText(f"Profils: {count}")
    
    def scan_finished(self, success, message):
        """
        NOUVELLE méthode appelée quand le scan se termine
        """
        self.scan_running = False
        self.scan_thread = None
        
        # Rétablir les boutons
        self.btn_start_serpentin.setEnabled(True)
        self.btn_cancel_serpentin.setEnabled(False)
        
        self.log("\n" + "=" * 60)
        self.log(message)
        self.log("=" * 60)
        
        if success:
            self.log("✅ Le scan est terminé avec succès!")
        else:
            self.log("⚠️ Le scan s'est terminé avec des problèmes")


# ============================================================================
# WIDGETS UI À AJOUTER
# ============================================================================

"""
Dans setup_serpentin_tab(), ajoutez ces widgets:

# Barre de progression
self.progress_bar_serpentin = QProgressBar()
self.progress_bar_serpentin.setRange(0, 100)
layout.addWidget(self.progress_bar_serpentin)

# Compteur de profils
self.lbl_profile_count = QLabel("Profils: 0")
self.lbl_profile_count.setStyleSheet("font-size: 14pt; font-weight: bold;")
layout.addWidget(self.lbl_profile_count)

# Boutons
btn_layout = QHBoxLayout()
btn_layout.addWidget(self.btn_start_serpentin)
btn_layout.addWidget(self.btn_cancel_serpentin)
layout.addLayout(btn_layout)
"""


# ============================================================================
# TEST STANDALONE
# ============================================================================

if __name__ == "__main__":
    print("""
    ╔══════════════════════════════════════════════════════════╗
    ║  CORRECTIONS THREADING SCAN SERPENTIN                    ║
    ╠══════════════════════════════════════════════════════════╣
    ║                                                          ║
    ║  ✅ GUI reste réactif pendant le scan                    ║
    ║  ✅ Possibilité de jog manuel pendant capture           ║
    ║  ✅ Annulation en temps réel                            ║
    ║  ✅ Logs en temps réel                                  ║
    ║  ✅ Barre de progression                                ║
    ║                                                          ║
    ╠══════════════════════════════════════════════════════════╣
    ║  INTÉGRATION:                                            ║
    ║                                                          ║
    ║  1. Copier ScanSerpentinThread dans votre fichier       ║
    ║  2. Remplacer start_scan_serpentin() par la version     ║
    ║     threadée                                             ║
    ║  3. Ajouter les nouvelles méthodes                       ║
    ║  4. Ajouter les widgets UI                              ║
    ║                                                          ║
    ╚══════════════════════════════════════════════════════════╝
    """)
