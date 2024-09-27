bool received = false;

struct {
  int8_t flm;
  int8_t frm;
  int8_t blm;
  int8_t brm;
} vel;

struct {
  bool a;
  bool b;
  bool c;
} flajok;


void uart() {
  Serial.print("1269$");

  delay(1000/30);

  received = false;
}



void setup() {
  Serial.begin(115200);
}

void loop() {
  uart();
}
