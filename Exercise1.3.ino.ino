String input;
String msg = "Enter Operation (e.g 4 + 2): ";
String msg2 = "Invalid Operation! ";
float num1, num2, result;
char op;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
}
void loop() {
  Serial.println(msg);
  while (Serial.available() == 0) {}
  input = Serial.readStringUntil('\n');
  Serial.print("You entered: ");
  Serial.println(input);
  sscanf(input.c_str(), "%d %c %d", &num1, &op, &num2);

switch(op){
  case '+': result = num1 + num2;
  Serial.print("Result = "); Serial.println(result); break;
  
  case '-': result = num1 - num2;
  Serial.print("Result = "); Serial.println(result); break;
  case '*': result = num1 * num2;
  Serial.print("Result = "); Serial.println(result); break;
  case '/': if (num2 != 0) {
    result = num1 / num2;
    Serial.print("Result = "); Serial.println(result);
  }else {
    Serial.println("Error, Division by zero!"); 
    return;
  }
  break;
  case '%':  // Modulo operator
      if (num2 != 0) {
        result = fmod(num1, num2);
        Serial.print("Result = "); 
        Serial.println(result);
      } else {
        Serial.println("Error, Modulo by zero!"); 
        return;
      }
      break;
  default:
  Serial.println(msg2);
  break;

}
}
