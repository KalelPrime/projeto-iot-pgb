const int led = 8;  // Pino onde o LED está conectado

void setup() {
  pinMode(led, OUTPUT);  // Define o pino como saída
  digitalWrite(led, HIGH);  // Acende o LED
}

void loop() {
  // Nada aqui, pois o LED já acendeu no setup()
}
