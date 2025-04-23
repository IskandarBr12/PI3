import cv2
import serial
import time

# === CONFIGURAÇÕES ===
porta_serial = 'COM4'  # ajuste conforme necessário
baud_rate = 9600
camera_index = 0

# === INICIALIZA SERIAL ===
try:
    ser = serial.Serial(porta_serial, baud_rate, timeout=1)
    time.sleep(2)
    print("Conectado à porta serial.")
except Exception as e:
    print("Erro ao conectar na porta serial:", e)
    exit()

# === INICIALIZA CÂMERA ===
cap = cv2.VideoCapture(camera_index)
if not cap.isOpened():
    print("Erro ao acessar a câmera.")
    exit()

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Função de mapeamento simples + leve ajuste de calibração (ângulo/perspectiva)
def mapear_com_ajuste(val, min_in, max_in, min_out, max_out, eixo='x'):
    val_mapeado = (val - min_in) * (max_out - min_out) / (max_in - min_in) + min_out

    # Correção simples baseada na posição do eixo para simular perspectiva
    if eixo == 'x':
        # Ajusta leve para bordas ficarem mais precisas
        fator = 1.05 if val < max_in / 2 else 0.95
    else:
        # Simula efeito de profundidade
        fator = 1.1 if val < max_in / 2 else 0.9

    return int(max(min_out, min(max_out, val_mapeado * fator)))

# === CALLBACK DE CLIQUE ===
def clique(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        angulo_x = mapear_com_ajuste(x, 0, frame_width, 0, 180, eixo='x')
        angulo_y = mapear_com_ajuste(y, 0, frame_height, 0, 180, eixo='y')

        comando = f"X:{angulo_x},Y:{angulo_y}\n"
        print("Enviando:", comando.strip())

        if ser.is_open:
            try:
                ser.write(comando.encode('utf-8'))
                time.sleep(0.15)
            except serial.SerialException as e:
                print("Erro ao enviar comando:", e)
                print("Tentando reconectar...")
                try:
                    ser.close()
                    time.sleep(1)
                    ser.open()
                    print("Reconectado à porta serial.")
                except Exception as erro_reconectar:
                    print("Falha ao reconectar. Encerrando.")
                    cap.release()
                    cv2.destroyAllWindows()
                    exit()

# === JANELA E LOOP ===
cv2.namedWindow("Câmera")
cv2.setMouseCallback("Câmera", clique)

print("Pressione 'q' para sair.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("Erro ao capturar frame.")
        break

    cv2.imshow("Câmera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# === FINALIZAÇÃO ===
cap.release()
cv2.destroyAllWindows()
ser.close()
