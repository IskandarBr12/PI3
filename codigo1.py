import cv2
import serial
import time

# === CONFIGURAÇÕES ===
porta_serial = 'COM4'  # ajuste conforme necessário
baud_rate = 9600
camera_index = 0

# Dimensões reais da tábua
tamanho_real_cm = {
    "largura": 100,  # eixo X
    "altura": 50     # eixo Y
}

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
print(f"Resolução da câmera: {frame_width}x{frame_height}")

# === MAPEAMENTO DE PIXEL PARA CM ===
def pixel_para_cm(x_pixel, y_pixel, largura_px, altura_px):
    x_cm = (x_pixel / largura_px) * tamanho_real_cm["largura"]
    y_cm = (y_pixel / altura_px) * tamanho_real_cm["altura"]
    return x_cm, y_cm

# === CONVERSÃO DE CM PARA ÂNGULO DO SERVO (0°–180°) ===
def cm_para_angulo(x_cm, y_cm):
    angulo_x = int((x_cm / tamanho_real_cm["largura"]) * 180)
    angulo_y = int((y_cm / tamanho_real_cm["altura"]) * 180)
    return angulo_x, angulo_y

# === CALLBACK DE CLIQUE ===
def clique(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Converte clique para coordenada real em centímetros
        x_cm, y_cm = pixel_para_cm(x, y, frame_width, frame_height)
        print(f"Coordenada real: {x_cm:.1f} cm, {y_cm:.1f} cm")

        # Converte para ângulo de servo
        angulo_x, angulo_y = cm_para_angulo(x_cm, y_cm)
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

print("Clique na imagem para obter a posição. Pressione 'q' para sair.")
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
