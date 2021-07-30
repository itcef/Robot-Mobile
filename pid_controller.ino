// Constantes
#define R         (float) 0.032
#define L         (float) 0.135
#define FORWARD           1
#define BACKWARD          0
#define rIn1              4
#define rIn2              5
#define lIn1              6
#define lIn2              7
#define rEnable           10
#define lEnable           11
#define rEncoder          2
#define lEncoder          3
#define lDcGain           17
#define rDcGain            17
#define PCONTROLLER       2.5
#define goalsNr           3
#define ultraTrigger    12
#define ultraEcho       13
#define ledAlaa         8  

//Initialisation des variables globales
volatile float rMotorTicks = 0;
volatile float lMotorTicks = 0;
float dR=0.0, dL=0.0, dC=0.0;
float x=0.0, y=0.0, theta=0.0;
float v = 0.2, w = 0,  vR = 0,  vL = 0, eX = 0, eY = 0 , eTheta = 0;
float rGoals[][goalsNr]={{.25,0},{.75, .25},{1,1}};
float thetaGoal = 0;
float vmr = 0, vml = 0, Vmr = 0, Vml = 0 ;
float lVolt,rVolt;
float lSpeed =0.0, rSpeed=0.0;
float lControlAction=0.0,rControlAction=0.0;
int   currentGoal = 0;
long  duration, distanceM;


void setup() {
  // Direction de la roue droite et activation des pins
  pinMode(rIn1, OUTPUT);
  pinMode(rIn2, OUTPUT);
  pinMode(rEnable, OUTPUT);

  // Direction de la roue gauche et activation des pins
  pinMode(lIn1, OUTPUT);
  pinMode(lIn2, OUTPUT);
  pinMode(lEnable, OUTPUT);

  // Pins droite et gauche de l'encodeur IR avec pull up sur chaque PIN
  pinMode(rEncoder, INPUT);
  digitalWrite(rEncoder, HIGH);       
  pinMode(lEncoder, INPUT);
  digitalWrite(lEncoder, HIGH); 

  // Activation externe après une interruption par front montant
  attachInterrupt(digitalPinToInterrupt(rEncoder), doRightEncoder, RISING); 
  attachInterrupt(digitalPinToInterrupt(lEncoder), doLeftEncoder, RISING);

  // Pins du capteur ultrason 
  pinMode(ultraTrigger, OUTPUT); 
  pinMode(ultraEcho, INPUT);

  // Obstacles LED pour Alaa
  pinMode(ledAlaa, OUTPUT);

  Serial.begin(9600);  
 
}

// Mise en marche du moteur droit en avant avec une vitesse spécifique
void rightWheel(int rspeed){
  digitalWrite(rIn1, LOW);
  digitalWrite(rIn2, HIGH);
  analogWrite(rEnable, rspeed);
}

// Mise en marche du moteur gauche en avant avec une vitesse spécifique 
void leftWheel(int lSpeed){
    digitalWrite(lIn1, LOW);
    digitalWrite(lIn2, HIGH);
    analogWrite(lEnable, lSpeed);
}

// Saturation de la vitesse angulaire en raison de limitations matérielles
float omegaSatauration(float w){
  if (w > PI/2) 
    return PI/2;
  if (w < -PI/2)
    return -PI/2;
  return w;
}

// Fonction du contrôleur PID sur l'erreur de la vitesse angulaire 
void pidController(){
  eTheta = thetaGoal - theta;
  w = PCONTROLLER * eTheta ;
  w = omegaSatauration(w);
}

// Module odométrie pour mettre à jour la position du robot
void actualPos(){
  dR=( rMotorTicks / 8.0 ) * ( 2 * PI * R );        
  dL=( lMotorTicks / 8.0 ) * ( 2 * PI * R );
  dC=( dR + dL ) / 2;
  x = x + ( dC * cos ( theta ) );
  y = y + ( dC * sin ( theta ) );
  theta = theta + ( ( dR - dL ) / L );
  theta = atan2( sin( theta ), cos( theta ) );
}

// Mapping (w,v) vers (vR,vL) en utilisant le modèle de conduite differentiel
float kinematics(){
  vR = v + ( w * L/2 );
  vL = v - ( w * L/2 );
}

// La boucle principale du programme
void loop() {
  
   // Si le robot n'est pas encore arrivé à destination
   if (((rGoals[currentGoal][0] - x)> 0.1) || (rGoals[currentGoal][1] - y)>.1){
    
    /* Code du capteur ultrason */
    
    digitalWrite(ultraTrigger, LOW);
    delayMicroseconds(2);
 
    digitalWrite(ultraTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultraTrigger, LOW);
    
    duration = pulseIn(ultraEcho, HIGH);
    // Calcul de la distance
    distanceM= (duration*0.034/2);

    //Tourne vers la droite si obstacle
    if(distanceM <= 0.15){
      digitalWrite(ledAlaa,HIGH);
      leftWheel(180);
      rightWheel(100);
      delay(50);
      }
      digitalWrite(ledAlaa,LOW);
  

    // Mise à jour de l'angle de destination  
    thetaGoal = atan2(rGoals[currentGoal][1]-y, rGoals[currentGoal][0]-x);

    // Génération de l'action de commande
    pidController();
    
    // Obtention de vR, vL par la fonction cinématique
    kinematics();
   
    // Vitesse désirée de la roue droite en (rad/s)
    rSpeed = vR / R;
    
    // Vitesse désirée de la roue droite en (RPM)
    rSpeed *= 30.0/PI;
     
    // Vitesse désirée de la roue gauche en (rad/s)
    lSpeed = vL / R;
    
    // Vitesse désirée de la roue gauche en (RPM)
    lSpeed *= 30.0/PI;
    
    // Calcul de la tension désirée de la roue gauche 
    lVolt = ( lSpeed / lDcGain );

    // Calcul de la tension désirée de la roue droite
    rVolt = ( rSpeed / rDcGain );
  
    // Mapping de l'action de commande (0~5.2)volt vers (0~255)
    lControlAction=map(lVolt,0,5.2,0,255);

    // Mapping de l'action de commande (0~5.2)volt vers (0~255)
    rControlAction=map(rVolt,0,5.2,0,255);
    
    //Application de l'action de commande à la roue gauche
    rightWheel(rControlAction-15);
    
    //Application de l'action de commande à la roue droite
    leftWheel(lControlAction);
  
    // Calcul de la position actuelle du robot en (m)
    actualPos();

    Serial.print(x);
    Serial.print(" , ");
    Serial.println(y);  

    }
    // Le robot est arrivé à destination mais pas la destination finale
    else{
      if (currentGoal != (goalsNr-1)){
        // Arrêt du robot et déplacement vers la destination suivante
        rightWheel(0);
        leftWheel(0);
        delay(500);
        currentGoal++;
      }
      // Le robot a atteint sa destination finale
       else{        
        rightWheel(0);
        leftWheel(0);
       }
    }
    
  // Réinitialisation des deux moteurs 
  rMotorTicks = 0;
  lMotorTicks = 0;

  delay(100);
}

void doRightEncoder(){
    rMotorTicks++;    
}


void doLeftEncoder(){
    lMotorTicks++;    
}