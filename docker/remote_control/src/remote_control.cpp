#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <curses.h>
#include <mutex>

#include "hardwareparameters.h"
#include "hardwareglobalinterface.h"

using namespace std::chrono_literals;

const auto R_ID_DEFAULT = 2;
std::unique_ptr<HardwareParameters> hp;

std::mutex pressedMTX;
std::unique_ptr<char> pressed;

void kbhit()
{
  int ch = 0;
  initscr();
  noecho();
  while((char)ch != 'q'){
    ch = getch();

    if (ch != ERR) {
      std::unique_lock<std::mutex> lock(pressedMTX);
      //printw("Key pressed! It was: %d\n", ch);
      //refresh();
      *pressed = (char)ch;
    } else {
      //printw("No key pressed yet...\n");
      //refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}


void prova(){
  HardwareGlobalInterface::getInstance().robotOnVelControl();
  char key_p;
  int v = 0;
  int omega = 0;
  double V_MAX = 0.1;
  double O_MAX = 0.1;
  bool terminate = false;

  double v_c = 0., omega_c = 0.;

  double V_MIN_LIMIT = 0.1;
  double V_MAX_LIMIT = 1;
  double O_MIN_LIMIT = 0.1;
  double O_MAX_LIMIT = 1;
  double ACC_V = 0.1;
  double ACC_OMEGA = 1.0;
  double DEC_V = 0.5;
  double DEC_OMEGA = 2.0;
  double STEP = 0.1;
  double A_LAT = 2.0;
  
  auto last = std::chrono::system_clock::now();
  /* Check for events */
  while(!terminate){
    bool prn = false;
    {
      std::unique_lock<std::mutex> lock(pressedMTX);
      key_p = *pressed;
      *pressed = 0;
    }
    /* Look for a keypress */
    switch( key_p ){
      case '4':
      case 'D':
        omega = 1;
        break;
      case '6':
      case 'C':
        omega = -1;
        break;
      case '8':
      case 'A':
        v = 1;
        break;
      case '2':
      case 'B':
        v = -1;
        break;
      case '5':
      case 's':
        omega = 0;
        v = 0;
        break;
      case 'w':
        V_MAX += STEP;
        V_MAX = std::min(V_MAX, V_MAX_LIMIT);
        prn = true;
        break;
      case 'x':
        V_MAX -= STEP;
        V_MAX = std::max(V_MAX, V_MIN_LIMIT);
        prn = true;
        break;
      case 'a':
        O_MAX -= STEP;
        O_MAX = std::max(O_MAX, O_MIN_LIMIT);
        prn = true;
        break;
      case 'd':
        O_MAX += STEP;
        O_MAX = std::min(O_MAX, O_MAX_LIMIT);
        prn = true;
        break;
      case 'q':
        terminate = true;
        break;
      default:
        break;
    } 
    clear();
    move(0,0);
    printw("*** USE q TO STOP THE REMOTE CONTROLLER ***\nUse w to increase v\nUse x to decrease v\nUse d to increase omega\nUse a to decrease omega\nUse 8 or arrow up to move forward\nUse 2 or arrow down to move backwards\nUse 4 or arrow left to curve left\nUse 6 or arrow right to curve right\nUse 5 or s to stop the robot\n*******************************************\n");
    printw("V_MAX: %f O_MAX: %f\n", V_MAX, O_MAX);

    auto curr = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = curr - last;
    double dT = diff.count();
    last = curr;

    if (omega==-1) {
      if (omega_c > -O_MAX) {
        omega_c -= (omega_c>0?DEC_OMEGA:ACC_OMEGA) * dT;
        omega_c = std::max(omega_c, -O_MAX);
      }
    }
    else if (omega==1) {
      if (omega_c < O_MAX) {
        omega_c += (omega_c<0?DEC_OMEGA:ACC_OMEGA) * dT;
        omega_c = std::min(omega_c, O_MAX);
      }
    }
    else {
      if (omega_c<0) {
        omega_c += DEC_OMEGA * dT;
        omega_c = std::min(omega_c, 0.);
      }
      else if (omega_c>0) {
        omega_c -= DEC_OMEGA * dT;
        omega_c = std::max(omega_c, 0.);
      }
    }

    double V_MAX_N = std::min(A_LAT/std::abs(omega_c), V_MAX);

    if (v==-1) {
      if (v_c > -V_MAX_N) {
        v_c -= (v_c>0?DEC_V:ACC_V) * dT;
        v_c = std::max(v_c, -V_MAX_N);
      }
      else if (v_c < -V_MAX_N) {
        v_c += DEC_V * dT;
        v_c = std::min(v_c, -V_MAX_N);
      }
    }
    else if (v==1) {
      if (v_c < V_MAX_N) {
        v_c += (v_c<0?DEC_V:ACC_V) * dT;
        v_c = std::min(v_c, V_MAX_N);
      }
      else if (v_c > V_MAX_N) {
        v_c -= DEC_V * dT;
        v_c = std::max(v_c, V_MAX_N);
      }
    }
    else {
      if (v_c<0) {
        v_c += DEC_V * dT;
        v_c = std::min(v_c, 0.);
      }
      else if (v_c>0) {
        v_c -= DEC_V * dT;
        v_c = std::max(v_c, 0.);
      }
    }
    HardwareGlobalInterface::getInstance().vehicleMove(v_c,omega_c);
    printw("dT: %f\n", dT);
    printw("v_c: %f omega_c: %f\n", v_c, omega_c);
    refresh();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  endwin();
  HardwareGlobalInterface::getInstance().robotOff();
}




int main(int argc, char * argv[])
{
  pressed = std::make_unique<char>();
  hp = std::make_unique<HardwareParameters>(R_ID_DEFAULT);
  HardwareGlobalInterface::initialize(hp.get());

  std::thread remote_control(prova);
  std::thread key_listener(kbhit);

  remote_control.join();
  key_listener.join();

  return 0;
}

