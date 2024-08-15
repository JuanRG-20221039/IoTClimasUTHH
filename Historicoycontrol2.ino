#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>
#include <WiFiManager.h>
#include <HardwareSerial.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <IRremote.h>

#include "IrCodigos.h"

// Definiciones de pines y configuraciones
#define DHTPIN 18
#define PIRPIN 5
#define DHTTYPE DHT11
#define LEDPIN 13
#define NUM_LECTURAS 600 // 5 minutos / 0.5 segundos = 600 lecturas
#define IR_PIN 19

// Variables globales
WiFiManager wifiManager;
DHT dht(DHTPIN, DHTTYPE);
HardwareSerial mySerial(2);

// Variables para la gestión de tiempo
unsigned long previousMillis = 0;
const long interval = 5000; // Intervalo en milisegundos

// Variables para almacenamiento de datos
String macAddress;
int id_iot = -1;
int id_placa_principal = -1;
int id_placa_secundaria = -1;
int IdVinculacionIOT = -1;
bool is_principal = false;
String claveCodigo;

// Bandera para controlar el estado de las consultas
bool consulta_id_iot_exitosa = false;
bool consulta_tipo_placa_exitosa = false;

//FUNCION SETUP PARA INICIAR EL VARABLES DEL PROGRAMA
void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // Inicializa mySerial // 16 - TX / 17 - RX
  pinMode(LEDPIN, OUTPUT);
  dht.begin();
  IrSender.begin(IR_PIN);

  // Iniciar WiFi Manager
  wifiManager.autoConnect("ESP32Config");

  // Obtener y guardar MAC Address
  macAddress = WiFi.macAddress();
  Serial.println("MAC de la Placa: " + macAddress);

  // Intentar obtener id_iot
  obtenerIdIot();
}

//LOOP DONDE SE EJECUTARA EL PROGRAMA
void loop() {
  unsigned long currentMillis = millis();

  // Realizar acciones cada intervalo definido
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Si no se ha obtenido id_iot, intentar obtenerlos nuevamente
    if (!consulta_id_iot_exitosa) {
      obtenerIdIot();
      return;
    }
    // Si no se ha obtenido tipo de placa, intentar obtenerlos nuevamente
    if (!consulta_tipo_placa_exitosa) {
      obtenerTipoPlaca();
      return;
    }

    // Leer datos de sensores
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println("Error al leer el DHT11");
      return;
    }

    // Construir el mensaje a enviar
    String mensaje = "Temperatura: " + String(t) + "C, Humedad: " + String(h) + "%";

    // Verificar estado del PIR
    int presenciaVal = 2; //leerPIR();
    
    mensaje += ", Presencia: " + String(presenciaVal);

    // Si la placa es principal, realiza la inserción de los datos en la base de datos mediante la API
    // Si la placa es secundaria, envía los datos mediante puerto serie a la placa principal
    if (is_principal) {
      Serial.println("Data: " + mensaje);

      // Consultar estado del clima y controlar LED
      int estadoActClima = consultarEstadoClima();

      // LLAMAR A LA NUEVA FUNCIÓN AQUÍ
      registrarHistorico(IdVinculacionIOT, presenciaVal, h, t, estadoActClima);

      // Procesar mensajes recibidos de la placa secundaria
      if (mySerial.available() > 0) {
        String mensaje_recibido = mySerial.readStringUntil('\n');
        Serial.println("Mensaje recibido de Placa secundaria: " + mensaje_recibido);
      } else {
        Serial.println("Error al comunicarse con la placa secundaria");
      }

    } else {
      // Si la placa es secundaria, enviar datos a la placa principal por puerto serie
      mySerial.println(mensaje);
      Serial.println(mensaje); // Imprime el mensaje a enviar 
    }
  }
}

// Función para obtener id_iot desde el servidor
void obtenerIdIot() {
  HTTPClient http;

  String url = "http://192.168.145.5:8000/iot/mac/" + macAddress;
  Serial.println("Consultando URL: " + url);

  http.begin(url);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println("Respuesta del servidor:");
    Serial.println(payload);

    // Parsear el JSON para obtener id_iot
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    if (doc.containsKey("Id_iot")) {
      id_iot = doc["Id_iot"].as<int>();
      consulta_id_iot_exitosa = true;

      

      // Consultar tipo de placa (principal o secundaria) basado en id_iot
      obtenerTipoPlaca();
    } else {
      Serial.println("Dispositivo IoT no encontrado, reintentando...");
      consulta_id_iot_exitosa = false;
    }
  } else {
    Serial.print("Error en la consulta HTTP: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

// Función para obtener el tipo de placa (principal o secundaria) desde el servidor
void obtenerTipoPlaca() {
  HTTPClient http;

  String url = "http://192.168.145.5:8000/vinculacion/modulo/" + String(id_iot);
  Serial.println("Consultando URL: " + url);

  http.begin(url);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println("Respuesta del servidor:");
    Serial.println(payload);

    // Parsear el JSON para obtener los IDs de las placas principal y secundaria
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    if (doc.containsKey("vinculacion")) {
      id_placa_principal = doc["vinculacion"]["Id_placa_principal"].as<int>();
      id_placa_secundaria = doc["vinculacion"]["Id_Placa_secundaria"].as<int>();
      IdVinculacionIOT = doc["vinculacion"]["Id_vinculacion_iot"].as<int>();

      obtenerModeloClima(IdVinculacionIOT);

      // Determinar si la placa es principal o secundaria
      if (id_placa_principal == id_iot || id_placa_secundaria == id_iot) {
        is_principal = (id_placa_principal == id_iot);
        consulta_tipo_placa_exitosa = true;
        Serial.println("La placa es " + String(is_principal ? "principal" : "secundaria"));
      } else {
        Serial.println("Error: id_iot no coincide con ninguna placa principal o secundaria.");
        consulta_tipo_placa_exitosa = false;
      }
    } else {
      Serial.println("Módulo IoT no encontrado, reintentando...");
      consulta_tipo_placa_exitosa = false;
    }
  } else {
    Serial.print("Error en la consulta HTTP: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

// Función para consultar y actualizar el estado del clima y controlar el LED
int consultarEstadoClima() {
  HTTPClient http;

  String url = "http://192.168.145.5:8000/iot/mac/" + macAddress;
  Serial.println("Consultando URL para Estado_clima: " + url);

  http.begin(url);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println("Respuesta del servidor para Estado_clima:");
    Serial.println(payload);

    // Parsear el JSON para obtener Estado_clima
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    if (doc.containsKey("Estado_clima")) {
      int estadoClima = doc["Estado_clima"].as<int>();
      if (estadoClima == 1) {
        digitalWrite(LEDPIN, HIGH); // Encender LED en pin 13
        Serial.println("LED Encendido");
        enviarCodigoIR(claveCodigo, 1);
        return 1;
      } else if (estadoClima == 0) {
        digitalWrite(LEDPIN, LOW); // Apagar LED en pin 13
        Serial.println("LED Apagado");
        enviarCodigoIR(claveCodigo, 0);
        return 0;
      } else {
        Serial.println("Estado_clima desconocido");
      }
    } else {
      Serial.println("Estado_clima no encontrado en la respuesta JSON");
    }
  } else {
    Serial.print("Error en la consulta HTTP: ");
    Serial.println(httpResponseCode);
  }

  http.end();
  return -1; // Retornar un valor por defecto si no se obtuvo el estado
}

//Funcion para enviar el mensaje por IR
void enviarCodigoIR(const String& claveCodigo, int estado) {
  const uint16_t* codigo;
  int rowData=-1;

  IrCodigos::getCodigoIR(claveCodigo, estado, codigo, rowData);
  if (rowData == -1) {
    Serial.println("Código no reconocido");
    return;
  }

  // Enviar el código IR
  IrSender.sendRaw(codigo, rowData, 38);
  Serial.println("Mensaje IR enviado");
}

// Función para leer el valor del PIR durante 5 minutos y devolver el resultado
int leerPIR() {
  int lecturas[NUM_LECTURAS];
  int suma = 0;
  
  for (int i = 0; i < NUM_LECTURAS; i++) {
    lecturas[i] = digitalRead(PIRPIN);
    delay(500); // Esperar 0.5 segundos entre lecturas
  }

  for (int i = 0; i < NUM_LECTURAS; i++) {
    suma += lecturas[i];
  }

  return (suma > 0) ? 1 : 0;
}

// Función para registrar histórico
void registrarHistorico(int idVinculacionIot, int presenciaPersonas, float humedadValue, float temperaturaValue, int estadoClima) {
  HTTPClient http;

  String url = "http://192.168.145.5:8000/historico-iot";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Crear JSON para enviar
  DynamicJsonDocument doc(1024);
  doc["Id_vinculacion_iot"] = idVinculacionIot;
  doc["Presencia_personas"] = presenciaPersonas;
  doc["Humedad_value"] = humedadValue;
  doc["Temperatura_value"] = temperaturaValue;
  doc["Estado_clima"] = estadoClima;

  String jsonData;
  serializeJson(doc, jsonData);

  int httpResponseCode = http.POST(jsonData);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Respuesta del servidor para registro histórico:");
    Serial.println(response);

    // Llamar a la función para actualizar los datos de la placa principal
    actualizarDatosPlacaPrincipal(presenciaPersonas, humedadValue, temperaturaValue, estadoClima);
    
  } else {
    Serial.print("Error en la consulta HTTP para registro histórico: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void actualizarDatosPlacaPrincipal(int presenciaPersonas, float humedadValue, float temperaturaValue, int estadoClima) {
  HTTPClient http;
  
  String url = "http://192.168.145.5:8000/iot/estado_clima/" + String(id_placa_principal);
  Serial.println("Actualizando datos de la placa principal en URL: " + url);

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Crear JSON para enviar
  DynamicJsonDocument doc(1024);
  doc["Presencia_personas"] = presenciaPersonas;
  doc["Humedad_value"] = humedadValue;
  doc["Temperatura_value"] = temperaturaValue;
  doc["Estado_clima"] = estadoClima;

  String jsonData;
  serializeJson(doc, jsonData);

  int httpResponseCode = http.PUT(jsonData);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Respuesta del servidor para actualización de la placa principal:");
    Serial.println(response);
  } else {
    Serial.print("Error en la consulta HTTP para actualización de la placa principal: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

// Función para obtener el modelo del clima y la marca
void obtenerModeloClima(int IdVinculacionIOT) {
  HTTPClient http;
  
  String url = "http://192.168.145.5:8000/climas/vinculacion/" + String(IdVinculacionIOT);
  Serial.println("Consultando URL para Modelo_clima: " + url);

  http.begin(url);
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println("Respuesta del servidor para Modelo_clima:");
    Serial.println(payload);

    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    if (doc.containsKey("Id_marca")) {
      int idMarca = doc["Id_marca"].as<int>();
      obtenerClaveMarca(idMarca); // Obtener la clave de la marca
    } else {
      Serial.println("Id_marca no encontrado en la respuesta JSON");
    }
  } else {
    Serial.print("Error en la consulta HTTP para Modelo_clima: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

// Función para obtener la clave de la marca
void obtenerClaveMarca(int idMarca) {
  HTTPClient http;
  String url = "http://192.168.145.5:8000/codigosClima/marca/" + String(idMarca);
  Serial.println("Consultando URL para Clave: " + url);

  http.begin(url);
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println("Respuesta del servidor para Clave:");
    Serial.println(payload);

    // Crear un documento JSON para analizar la respuesta
    DynamicJsonDocument doc(1024);

    // Parsear la respuesta JSON
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("Error al parsear JSON: ");
      Serial.println(error.c_str());
      return;
    }

    // Verificar si el JSON es un array
    if (doc.is<JsonArray>()) {
      JsonArray array = doc.as<JsonArray>();

      // Iterar sobre los objetos del array
      for (JsonObject obj : array) {
        if (obj.containsKey("Clave")) {
          claveCodigo = obj["Clave"].as<String>();
          Serial.println("Clave de la marca: " + claveCodigo);
          return;
        }
      }
      Serial.println("Clave no encontrada en la respuesta JSON");
    } else {
      Serial.println("Respuesta JSON no es un array");
    }
  } else {
    Serial.print("Error en la consulta HTTP para Clave: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}