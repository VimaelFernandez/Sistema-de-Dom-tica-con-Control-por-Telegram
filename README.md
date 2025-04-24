<h1>Sistema de Domótica con Control por Telegram</h1>

<h3> 🏠Visión General del Proyecto<h3/>
<p>Este repositorio contiene un sistema completo de domótica que permite controlar luces y ventiladores de forma remota mediante Telegram, usando ESP-NOW para comunicación local entre dispositivos ESP.
</p>
  
<h3>📌 Características Principales</h3>
<ul>
  <li>Integración con Bot de Telegram – Control remoto mediante comandos</li>
  <li>Comunicación ESP-NOW – Protocolo inalámbrico rápido y eficiente</li>
  <li>Control Híbrido – Funciona tanto por Telegram como por interruptores físicos</li>
  <li>Automatización por Temperatura – El ventilador ajusta su velocidad según la temperatura</li>
  <li>Soporte para Múltiples Dispositivos – Control central con ESP32 y nodos ESP8266</li>
</ul>
<h3>📋 Arquitectura del Sistema</h3>
El sistema consta de tres componentes principales:

Controlador Central (ESP32)

Actúa como puente entre Telegram y los dispositivos ESP-NOW

Ejecuta un bot de Telegram para la interacción

Gestiona la comunicación con los nodos ESP8266

Controlador del Ventilador (ESP8266)

Maneja un ventilador de 3 velocidades

Modo automático (basado en temperatura) y manual (control por Telegram)

Reporta datos de temperatura al controlador central

Controlador de Luz (ESP8266)

Controla una luz (relé/LED)

Soporta interruptor físico y control remoto (con prioridad al interruptor)

🛠 Requerimientos de Hardware
Controlador Central (ESP32)
Placa ESP32

Conexión WiFi

(Opcional) Pantalla OLED para estado local

Controlador del Ventilador (ESP8266)
ESP8266 (NodeMCU/Wemos D1 Mini)

Sensor de temperatura DS18B20

Módulo de relé (para control del ventilador)

Ventilador de 3 velocidades

Controlador de Luz (ESP8266)
ESP8266 (NodeMCU/Wemos D1 Mini)

Módulo de relé (para control de luz)

Interruptor físico

📡 Protocolo de Comunicación
El sistema usa ESP-NOW para comunicación eficiente entre dispositivos:

Dispositivo	Rol	Dirección MAC
ESP32 (Central)	Puente Telegram ↔ ESP-NOW	EC:64:C9:CD:DB:48
ESP8266 (Ventilador)	Control de temperatura y ventilador	40:22:D8:77:83:8C
ESP8266 (Luz)	Control de interruptor de luz	40:22:D8:77:83:8D
📶 Proceso de Emparejamiento
El ESP32 inicia en modo AP+STA y espera conexiones.

Los nodos ESP8266 escanean canales WiFi y envían solicitudes de emparejamiento.

Una vez emparejados, se comunican via ESP-NOW sin necesidad de WiFi.

🤖 Comandos del Bot de Telegram
Control del Ventilador
Comando	Acción
/Velocidad_1	Ajusta a Velocidad 1 (Baja)
/Velocidad_2	Ajusta a Velocidad 2 (Media)
/Velocidad_3	Ajusta a Velocidad 3 (Alta)
/Apagado	Apaga el ventilador
/Automatico	Activa modo automático
/Estado	Consulta estado actual
Control de Luz
Comando	Acción
/Cuarto Encendido	Enciende la luz
/Cuarto Apagado	Apaga la luz
⚙️ Configuración e Instalación
1. Cargar el Código en los Dispositivos
ESP32 (Control Central) → Usar ESP32_TelegramBot_Controller.ino

ESP8266 (Control del Ventilador) → Usar ESP8266_Fan_Controller.ino

ESP8266 (Control de Luz) → Usar ESP8266_Light_Controller.ino

2. Configurar WiFi y Telegram
Reemplazar ssid, password y BOT_TOKEN en el código del ESP32.

Asegurarse de que todos los dispositivos estén en la misma red WiFi durante el emparejamiento.

3. Conexiones de Hardware
Control del Ventilador:

Conectar DS18B20 a GPIO14 (D5)

Pines de control del relé: GPIO12 (D6), GPIO13 (D7), GPIO15 (D8)

Control de Luz:

Interruptor físico en GPIO2 (D4)

Control del relé en GPIO0 (D3)

📊 Comportamiento del Sistema
Lógica del Ventilador
Modo Manual: Velocidad ajustada por comandos de Telegram.

Modo Automático: Ajusta la velocidad según la temperatura:

≤28°C → Velocidad 1

29-31°C → Velocidad 2

>31°C → Velocidad 3

Lógica de la Luz
El interruptor físico tiene prioridad → Si está activado, ignora comandos remotos.

Control remoto funciona solo cuando el interruptor está apagado.

🔧 Solución de Problemas
Problema	Solución
Fallo en el emparejamiento ESP-NOW	Verificar direcciones MAC y canal WiFi
El bot de Telegram no responde	Revisar BOT_TOKEN y conexión WiFi
El ventilador/luz no enciende	Verificar cableado y configuración de GPIO
📜 Licencia
Este proyecto es de código abierto bajo la Licencia MIT.

📌 Estructura del Repositorio
📁 Domotica-Telegram-ESP-NOW/
├── 📁 ESP32_Control_Central/      # Controlador principal (Telegram ↔ ESP-NOW)
├── 📁 ESP8266_Control_Ventilador/ # Control del ventilador (temp + remoto)
├── 📁 ESP8266_Control_Luz/        # Control de luz (interruptor + remoto)
├── 📄 README.md                   # Esta documentación
└── 📄 LICENSE                     # Licencia MIT
🚀 Mejoras Futuras
Integración con MQTT para registro en la nube

Panel de Control Web para gestión local

Monitoreo de Energía con sensores

🎯 Conclusión
Este sistema ofrece una solución económica, eficiente y escalable para domótica usando ESP32/ESP8266 + Telegram + ESP-NOW. Ideal para habitaciones inteligentes, proyectos IoT y automatización DIY.

🔗 Enlace al Repositorio: https://github.com/tuusuario/Domotica-Telegram-ESP-NOW

💡 ¡Contribuciones son bienvenidas! Abre issues o PRs para mejoras.
