
//подключаем бибилиотеку работы с панелью
#include "Nextion.h"
#include  "EEPROMex.h"
#include "Arduino.h"
#include "ModBusRTU.h"
#include "DRV_SD.h"
#include "DRV_Lib.h"
#include "TimerOne.h"
#include <avr/wdt.h>
int PolyMix_ID = 8281;  //условный ID установки, не более 32000, например [НОМЕР ПРОЕКТ(828 - Вохутга)] & [НОМЕР УСТАНОВКИ(Флокулянт - 1)]
int reagent = 0; //0 - SA, Flok, PAU; 1 - Soda

//СОЗДАЁМ ЭКЗЕМПЛЯРЫ УСТРОЙСТВ
  //СОЗДАЁМ ДВИГАТЕЛИ
    DRV_MD M1;  //мешалка 1
    DRV_MD M2;  //машалка 2
    DRV_MD M3;  //мешалка 
    DRV_MD TH;  //транспортный шнек
    DRV_MD DS;  //вибратор на буфере
    DRV_MD TS;  //вибратор на бункере
    DRV_MD KC;  //клапан подачи воды

  //СОЗДАЁМ ДИСКРЕТЫЕ ДАТЧИКИ
    DRV_DI LLS; //низкий уровень буфера
    DRV_DI HLS; //высокий уровень буфера
    DRV_DI ALS; //низкий уровень бункера
    DRV_DI Flow_RM;//импульс от расходомера
    DRV_DI Start_REQ;//разрешение на работу
    DRV_DI Block;//внешняя авария
    DRV_DI LS;
    DRV_DI tDI;

  //СОЗДАЁМ ДИСКРЕТНЫЕ ВЫХОДА
    DRV_DO LED_Start;
    DRV_DO LED_Alarm;
    DRV_DO LED_Dry_Run;
    DRV_DO LED_Reagent;

  //СОЗДАЁМ АНАЛОГОВЫЕ ДАТЧИКИ
    DRV_AI LE;  //датчик уровня в ёмкости

  //СОЗДАЁМ ИМПУЛЬСНЫЕ РАСХОДОМЕРЫ
    DRV_Flow RM;

  //СОЗДАЁМ ШАГОВЫЕ ПРИВОДЫ
    DRV_SD SD;  //дозирующий шнек
    void SD_exe()
    {
      if (SD.Start == 1)
      {
        SD.exe();
      }
      if (SD.Start == 0)
      {
        PORTH = B00000000;
      }
    }

  //СОЗДАЁМ ТАЙМЕРЫ
    TON_m T_M1;
    TON_m T_M2;
    TON_m T_M3;
    TON_s T_DS_Work_ON;
    TON_s T_DS_Wait_ON;
    TON_s T_DS_Work_OFF;
    TON_m T_DS_Wait_OFF;
    TON_s T_LLS_delay;
    TON_m T_LLS_error;
    TON_s T_HLS_delay;
    TON_s T_TH_ON;
    TON_s T_TS_ON;
    TON_m T_TS_OFF;
    TON_m T_Alm[8]; //0 - сухой ход, 1 - перелив, 2 - нет протока, 3 - нет заполнения, 4 - буфер пуст, 5 - бункер пуст
    TON_s T_ModBus;
    TON_m T_Sett_Access;
    TON_m T_Sys_Access;
    TON_s T_KC_Delay;
    TON_m T_TH_Limit;

  // массив данных modbus
    uint16_t MB_Data[50];
    uint32_t number = 0;
    
  //Задаём ведомому адрес, последовательный порт, выход управления Tx
    int modbus_addr = 10; //адрес подключения по умолчанию
    int modbus_use = 0;   //0 - управление локальное, 1 = управление по сети
    int modbus_use_vis = 0;
    int modbus_status = 0;
    int modbus_status_vis = 0;
    int WatchDog = 0;
    int WD_in = 0;
    int WD_in_last = 0;
    int8_t state = 0;     //состояния обмена
    Modbus slave(modbus_addr, 1, 44);

  //БОКОВОЕ МЕНЮ
    uint8_t scr = 0;
    uint8_t scr_active = 0;
    int scr_update = 0;
    int scr_cycle = 0;

  //ОБРАБОТКА АВАРИЙ
    int any_alarm = 0;   //наличие любой аварии
    int any_alarm_vis = 0;
    int alarm_devices = 0;
    int error_link = 0;  //1 = обрыв связи
    int error_link_vis = 0;
    int work_alarms[9];  //0 - сухой ход, 1 - перелив, 2 - нет протока, 3 - нет заполнения, 4 - буфер пуст, 5 - бункер пуст, 6 - дозация, 7 - разрешение, 8 - внешняя блокировка
    int work_alarms_vis[9];
    //float dos_cnt_max = 3;

//ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
  //Управление
    int CMD_Start = 0;  //общий пуск
    int REQ_Start = 0;  //кнопка запуска
    int CMD_Restart = 0;//сохранение команды при перезагрузке
    int CMD_Restart_vis = 0;
    int CMD_Save = 0;
    int CMD_Save_vis = 0;
    int CMD_Status = 0;
    int CMD_Status_vis = 0;
    int LE_Ok = 0;
    int CMD_0_vis = 0;
    int CMD_1_vis = 0;
    int CMD_2_vis = 0;
    int CMD_3_vis = 0;
    int CMD_4_vis = 0;
    int CMD_5_vis = 0;
    int CMD_6_vis = 0;
    int CMD_7_vis = 0;
    float SP_C = 2;
    float SP_High = 95;
    float SP_Low = 35;  
    int Reset_All = 0;
    int loading = 0;
    int save_main = 0;
    int save_sett = 0;
    int save_sys = 0;
    int save_alarms = 0;
    int save_users = 0;
    int TS_REQ = 0;
  //Конфигурация мешалок и секций
    int mixers = 0;
    int mix_1_use = 1;
    int mix_1_use_vis = 0;
    int mix_2_use = 0;
    int mix_2_use_vis = 0;
    int mix_3_use = 0;
    int mix_3_use_vis = 0;
    int mix_12_use = 0;
    int mix_12_use_vis = 0;
    int mix_123_use = 0;
    int mix_123_use_vis = 0;
    int mix_2_speed = 0;
    int mix_2_speed_vis = 0;
    int sections = 0;
    int section_1 = 1;
    int sec_1_vis = 0;
    int section_2 = 0;
    int sec_2_vis = 0;
    int section_3 = 0;
    int sec_3_vis = 0;
  //Конфигурация вибраторов
    int DS_use = 0;
    int DS_use_vis = 0;
    int TH_use = 0;
    int TH_use_vis = 0;
    int TS_use = 0;
    int TS_use_vis = 0;
  //Калибровка дозатора
    float dos_c_distance = 5;
    float dos_c_speed = 10;
    float dos_c_mass = 250;
    float dos_c_source = 97;
    float dos_c_one = 0;
    float dos_c_impulse = 0;
    int dos_c_start = 0;
    int dos_c_start_vis = 0;
    int dos_c_cancel = 0;
    int dos_c_vancel_vis = 0;
    int dos_cnt_add = 0;
    int dos_cnt_sub = 0;
    int dos_cnt = 0;
    int dos_cnt_max = 5;
  //Расходомер
    float Result_Flow = 0;
  //Приготовление
    int LE_Step = 0;
    int Mix_Step = 0;
    int DS_Step = 0;
    int DS_S1 = 0;
    int DS_S2 = 0;
    int TH_Step = 0;
    int TS_Step = 0;
    float M2_Low_SP = 50.0;
    float M2_High_SP = 75.0;
    int kc_open = 0;
  //уровни доступа
    int sett_access = 0;
    int sys_access = 0;

//======> EEPROM
    //адреса переменных
        #define first_loading       0
      //Main = 0
        #define ee_SP_C             4
        #define ee_SP_High          8
        #define ee_SP_Low           12
      //Sett = 1
        #define ee_dos_c_speed      16
        #define ee_dos_c_distance   20
        #define ee_dos_c_mass       24
        #define ee_dos_c_source     28
        #define ee_T_M1_tSP         32
        #define ee_T_M2_tSP         36
        #define ee_T_M3_tSP         40
        #define ee_M1_use           44
        #define ee_M2_use           46
        #define ee_M3_use           48
        #define ee_M12_use          50
        #define ee_M123_use         52
        #define ee_DS_use           54
        #define ee_TH_use           56
        #define ee_TS_use           58
        #define ee_T_DS_Work_ON     60
        #define ee_T_DS_Wait_ON     64
        #define ee_T_DS_Work_OFF    68
        #define ee_T_DS_Wait_OFF    72
        #define ee_T_LLS_delay      76
        #define ee_T_LLS_error      80
        #define ee_T_HLS_delay      84
        #define ee_T_TH_ON          88
        #define ee_T_TS_ON          92
        #define ee_T_TS_OFF         96
        #define ee_T_KC_Delay       168
        #define ee_M2_Low_SP        170
        #define ee_M2_High_SP       174
        #define ee_T_TH_Limit       178
        #define ee_modbus_use       100
        #define ee_modbus_addr      102
      //Sys = 2
        #define ee_SD_SP_imp        104
        #define ee_SD_SP_ms         108
        #define ee_SD_Adapt         112
        #define ee_SD_Vector        114
        #define ee_LLS_Use          116
        #define ee_HLS_Use          118
        #define ee_ALS_Use          120
        #define ee_LLS_Inv          182
        #define ee_HLS_Inv          184
        #define ee_ALS_Inv          186
        #define ee_mix_2_speed      122
        #define ee_section_2        124
        #define ee_section_3        126
      //Alarm = 3
        #define ee_dos_cnt_max      128
        #define ee_LE_LLS           132
        #define ee_LE_HHS           136
        #define ee_T_Alm_0          140
        #define ee_T_Alm_1          144
        #define ee_T_Alm_2          148
        #define ee_T_Alm_3          152
        #define ee_T_Alm_4          156
        #define ee_T_Alm_5          160
        #define ee_CMD_Restart      164
        #define ee_CMD_Start        166

    //сохранение переменной в ЕЕПРОМ
      void ee_save_float(float value, int address)
      {
        if (value != EEPROM.readFloat(address))
        {
          EEPROM.writeFloat(address, value);
        }
      }
      void ee_save_int(int value, int address)
      {
        if (value != EEPROM.readInt(address))
        {
          EEPROM.writeInt(address, value);
        }
      }

    //чтение переменной из ЕЕПРОМ
      float ee_load_float(int address)
      {
        return EEPROM.readFloat(address);
      }
      int ee_load_int(int address)
      {
        return EEPROM.readInt(address);
      }

    //чтение всех данных при загрузке
      void ee_loading()
      {
        loading = ee_load_int(first_loading);

        if (loading == 1)
        {
          //Main = 0
            SP_C                  = ee_load_float(ee_SP_C);
            LE.HS                 = ee_load_float(ee_SP_High);
            LE.LS                 = ee_load_float(ee_SP_Low);

          //Sett = 1
            //Дозация
              dos_c_speed         = ee_load_float(ee_dos_c_speed);
              dos_c_distance      = ee_load_float(ee_dos_c_distance);
              dos_c_mass          = ee_load_float(ee_dos_c_mass);
              dos_c_source        = ee_load_float(ee_dos_c_source);
            //Мешалки
              T_M1.tSP            = ee_load_float(ee_T_M1_tSP);
              T_M2.tSP            = ee_load_float(ee_T_M2_tSP);
              T_M3.tSP            = ee_load_float(ee_T_M3_tSP);
              mix_1_use           = ee_load_int(ee_M1_use);
              mix_2_use           = ee_load_int(ee_M2_use);
              mix_3_use           = ee_load_int(ee_M3_use);
              mix_12_use          = ee_load_int(ee_M12_use);
              mix_123_use         = ee_load_int(ee_M123_use);
            //Вибраторы и буфер
              DS_use              = ee_load_int(ee_DS_use);
              TH_use              = ee_load_int(ee_TH_use);
              TS_use              = ee_load_int(ee_TS_use);
              T_DS_Work_ON.tSP    = ee_load_float(ee_T_DS_Work_ON);
              T_DS_Work_OFF.tSP   = ee_load_float(ee_T_DS_Work_OFF);
              T_DS_Wait_ON.tSP    = ee_load_float(ee_T_DS_Wait_ON);
              T_DS_Wait_OFF.tSP   = ee_load_float(ee_T_DS_Wait_OFF);
              T_TH_ON.tSP         = ee_load_float(ee_T_TH_ON);
              T_TS_ON.tSP         = ee_load_float(ee_T_TS_ON);
              T_TS_OFF.tSP        = ee_load_float(ee_T_TS_OFF);
              T_KC_Delay.tSP      = ee_load_int(ee_T_KC_Delay);
              M2_Low_SP           = ee_load_float(ee_M2_Low_SP);
              M2_High_SP          = ee_load_float(ee_M2_High_SP);
              T_TH_Limit.tSP      = ee_load_int(ee_T_TH_Limit);
            //ModBus RTU
              modbus_use          = ee_load_int(ee_modbus_use);
              modbus_addr         = ee_load_int(ee_modbus_addr);

          //Sys = 2
            //Шнековый дозатор
              SD.SP_imp           = ee_load_float(ee_SD_SP_imp);
              SD.SP_ms            = ee_load_float(ee_SD_SP_ms);
              SD.Adapt            = ee_load_int(ee_SD_Adapt);
              SD.Vector           = ee_load_int(ee_SD_Vector);
            //Датчики
              LLS.Use             = ee_load_int(ee_LLS_Use);
              HLS.Use             = ee_load_int(ee_HLS_Use);
              ALS.Use             = ee_load_int(ee_ALS_Use);
              LLS.Inv             = ee_load_int(ee_LLS_Use);
              HLS.Inv             = ee_load_int(ee_HLS_Inv);
              ALS.Inv             = ee_load_int(ee_ALS_Inv);
              mix_2_speed         = ee_load_int(ee_mix_2_speed);
              section_2           = ee_load_int(ee_section_2);
              section_3           = ee_load_int(ee_section_3);

          //Alarms = 3
            dos_cnt_max = ee_load_float(ee_dos_cnt_max);
            LE.LLS = ee_load_float(ee_LE_LLS);
            LE.HHS = ee_load_float(ee_LE_HHS);
            T_Alm[0].tSP = ee_load_float(ee_T_Alm_0);
            T_Alm[1].tSP = ee_load_float(ee_T_Alm_1);
            T_Alm[2].tSP = ee_load_float(ee_T_Alm_2);
            T_Alm[3].tSP = ee_load_float(ee_T_Alm_3);
            T_Alm[4].tSP = ee_load_float(ee_T_Alm_4);
            T_Alm[5].tSP = ee_load_float(ee_T_Alm_5);
            CMD_Restart = ee_load_int(ee_CMD_Restart);
            if (CMD_Restart == 1) {REQ_Start = ee_load_int(ee_CMD_Start); CMD_Save = REQ_Start;}
        }
      }

    //Запись данных при сохранении
      void ee_saving()
      {
        //Main = 0
          if (scr_active == 0 and scr_update == 1)
          {
            ee_save_float(SP_C, ee_SP_C);
            ee_save_float(LE.HS, ee_SP_High);
            ee_save_float(LE.LS, ee_SP_Low);

            if (CMD_Restart == 1) {ee_save_int(REQ_Start, ee_CMD_Start); CMD_Save = REQ_Start;}

            save_main = 1;
          }

        //Sett = 1
          if (scr_active == 1 and scr_update == 1)
          {
            //Дозация
              ee_save_float(dos_c_speed, ee_dos_c_speed);
              ee_save_float(dos_c_distance, ee_dos_c_distance);
              ee_save_float(dos_c_mass, ee_dos_c_mass);
              ee_save_float(dos_c_source, ee_dos_c_source);
            //Мешалки
              ee_save_float(T_M1.tSP, ee_T_M1_tSP);
              ee_save_float(T_M2.tSP, ee_T_M2_tSP);
              ee_save_float(T_M3.tSP, ee_T_M3_tSP);
              ee_save_int(mix_1_use, ee_M1_use);
              ee_save_int(mix_2_use, ee_M2_use);
              ee_save_int(mix_3_use, ee_M3_use);
              ee_save_int(mix_12_use, ee_M12_use);
              ee_save_int(mix_123_use, ee_M123_use);
            //Вибраторы и буфер
              ee_save_int(DS_use, ee_DS_use);
              ee_save_int(TH_use, ee_TH_use);
              ee_save_int(TS_use, ee_TS_use);
              ee_save_float(T_DS_Work_ON.tSP, ee_T_DS_Work_ON);
              ee_save_float(T_DS_Work_OFF.tSP, ee_T_DS_Work_OFF);
              ee_save_float(T_DS_Wait_ON.tSP, ee_T_DS_Wait_ON);
              ee_save_float(T_DS_Wait_OFF.tSP, ee_T_DS_Wait_OFF);
              ee_save_float(T_TH_ON.tSP, ee_T_TH_ON);
              ee_save_float(T_TS_ON.tSP, ee_T_TS_ON);
              ee_save_float(T_TS_OFF.tSP, ee_T_TS_OFF);
              ee_save_int(T_KC_Delay.tSP, ee_T_KC_Delay);
              ee_save_float(M2_Low_SP, ee_M2_Low_SP);
              ee_save_float(M2_High_SP, ee_M2_High_SP);
              ee_save_int(T_TH_Limit.tSP, ee_T_TH_Limit);
            //ModBus RTU
              ee_save_int(modbus_use, ee_modbus_use);
              ee_save_int(modbus_addr, ee_modbus_addr);

            save_sett = 1;
          }

        //Sys = 2
          if (scr_active == 2 and scr_update == 1)
          {
            //Настройка аварий
              ee_save_float(LE.LLS, ee_LE_LLS);
              ee_save_float(LE.HHS, ee_LE_HHS);
              ee_save_float(T_Alm[0].tSP, ee_T_Alm_0);
              ee_save_float(T_Alm[1].tSP, ee_T_Alm_1);
              ee_save_float(T_Alm[2].tSP, ee_T_Alm_2);
              ee_save_float(T_Alm[3].tSP, ee_T_Alm_3);
              ee_save_float(T_Alm[4].tSP, ee_T_Alm_4);
              ee_save_float(T_Alm[5].tSP, ee_T_Alm_5);
            //Датчики
              ee_save_int(LLS.Use, ee_LLS_Use);
              ee_save_int(HLS.Use, ee_HLS_Use);
              ee_save_int(ALS.Use, ee_ALS_Use);
              ee_save_int(LLS.Inv, ee_LLS_Inv);
              ee_save_int(HLS.Inv, ee_HLS_Inv);
              ee_save_int(ALS.Inv, ee_ALS_Inv);
              ee_save_int(mix_2_speed, ee_mix_2_speed);
              ee_save_int(section_2, ee_section_2);
              ee_save_int(section_3, ee_section_3);

            save_sys = 1;
          }

        //Alarms = 3
          if (scr_active == 3 and scr_update == 1)
          {
            ee_save_float(dos_cnt_max, ee_dos_cnt_max);
            
            ee_save_int(CMD_Restart, ee_CMD_Restart);

            save_alarms = 1;
          }

        if (scr_active == 7 and scr_update == 1)
        {
          //Шнековый дозатор
              ee_save_float(SD.SP_imp, ee_SD_SP_imp);
              ee_save_float(SD.SP_ms, ee_SD_SP_ms);
              ee_save_float(SD.Adapt, ee_SD_Adapt);
              ee_save_float(SD.Vector, ee_SD_Vector);

          save_users = 1;
        }
        //Контроль сохранения данных
          if (save_main == 1 and save_sett == 1 and save_sys == 1 and save_alarms == 1 and save_users == 1)
            {
              loading = 1;
              ee_save_int(loading, first_loading);
            }
      }

//=======> NEXTION CONFIG
  //Навигация
    //Main=0
      NexPage nex_0_page =          NexPage(0, 0, "Main");
      NexButton nex_0_menu_home =   NexButton(0, 1, "Key_Home");
      NexButton nex_0_menu_sett =   NexButton(0, 2, "Key_Sett");
      NexButton nex_0_menu_sys =    NexButton(0, 3, "Key_Sys");
      NexButton nex_0_menu_alarm =  NexButton(0, 4, "Key_Alarm");
      NexButton nex_0_menu_save =   NexButton(0, 5, "Key_Save");
    //Sett=1
      NexPage nex_1_page =          NexPage(1, 0, "Sett");
      NexButton nex_1_menu_home =   NexButton(1, 1, "Key_Home");
      NexButton nex_1_menu_sett =   NexButton(1, 2, "Key_Sett");
      NexButton nex_1_menu_sys =    NexButton(1, 3, "Key_Sys");
      NexButton nex_1_menu_alarm =  NexButton(1, 4, "Key_Alarm");
      NexButton nex_1_menu_save =   NexButton(1, 5, "Key_Save");
    //Sys=2
      NexPage nex_2_page =          NexPage(2, 0, "Sys");
      NexButton nex_2_menu_home =   NexButton(2, 1, "Key_Home");
      NexButton nex_2_menu_sett =   NexButton(2, 2, "Key_Sett");
      NexButton nex_2_menu_sys =    NexButton(2, 3, "Key_Sys");
      NexButton nex_2_menu_alarm =  NexButton(2, 4, "Key_Alarm");
      NexButton nex_2_menu_save =   NexButton(2, 5, "Key_Save");
    //alarms=3
      NexPage nex_3_page =          NexPage(3, 0, "Alarms");
      NexButton nex_3_menu_home =   NexButton(3, 1, "Key_Home");
      NexButton nex_3_menu_sett =   NexButton(3, 2, "Key_Sett");
      NexButton nex_3_menu_sys =    NexButton(3, 3, "Key_Sys");
      NexButton nex_3_menu_alarm =  NexButton(3, 4, "Key_Alarm");
      NexButton nex_3_menu_save =   NexButton(3, 5, "Key_Save");
      NexPicture nex_3_login =      NexPicture(3, 79, "p41");
    //sett_lock=4
      NexPage nex_4_page =          NexPage(4, 0, "SettLock");
      NexButton nex_4_menu_home =   NexButton(4, 1, "Key_Home");
      NexButton nex_4_menu_sett =   NexButton(4, 2, "Key_Sett");
      NexButton nex_4_menu_sys =    NexButton(4, 3, "Key_Sys");
      NexButton nex_4_menu_alarm =  NexButton(4, 4, "Key_Alarm");
      NexButton nex_4_menu_save =   NexButton(4, 5, "Key_Save");
      NexPicture nex_4_login =      NexPicture(4, 11, "p0");
      NexText nex_4_value =         NexText(4, 15, "t1");
    //sett_lock=5
      NexPage nex_5_page =          NexPage(5, 0, "SysLock");
      NexButton nex_5_menu_home =   NexButton(5, 1, "Key_Home");
      NexButton nex_5_menu_sett =   NexButton(5, 2, "Key_Sett");
      NexButton nex_5_menu_sys =    NexButton(5, 3, "Key_Sys");
      NexButton nex_5_menu_alarm =  NexButton(5, 4, "Key_Alarm");
      NexButton nex_5_menu_save =   NexButton(5, 5, "Key_Save");
      NexPicture nex_5_login =      NexPicture(5, 10, "p0");
      NexText nex_5_value =         NexText(5, 15, "t1");
    //pass_lock=6
      NexPage nex_6_page =          NexPage(6, 0, "PassLock");
      NexButton nex_6_menu_home =   NexButton(6, 1, "Key_Home");
      NexButton nex_6_menu_sett =   NexButton(6, 2, "Key_Sett");
      NexButton nex_6_menu_sys =    NexButton(6, 3, "Key_Sys");
      NexButton nex_6_menu_alarm =  NexButton(6, 4, "Key_Alarm");
      NexButton nex_6_menu_save =   NexButton(6, 5, "Key_Save");
      NexPicture nex_6_login =      NexPicture(6, 10, "p0");
      NexText nex_6_value =         NexText(6, 15, "t2");
    //user_control=7
      NexPage nex_7_page =          NexPage(7, 0, "UserControl");
      NexButton nex_7_menu_home =   NexButton(7, 1, "Key_Home");
      NexButton nex_7_menu_sett =   NexButton(7, 2, "Key_Sett");
      NexButton nex_7_menu_sys =    NexButton(7, 3, "Key_Sys");
      NexButton nex_7_menu_alarm =  NexButton(7, 4, "Key_Alarm");
      NexButton nex_7_menu_save =   NexButton(7, 5, "Key_Save");

  //Статусы
    //Main=0
      NexPicture nex_0_rs232 =      NexPicture(0, 6, "Icon_Update");
      NexPicture nex_0_link =       NexPicture(0, 7, "Icon_Link");
      NexPicture nex_0_start =      NexPicture(0, 8, "Icon_Start");
    //Sett=1
      NexPicture nex_1_rs232 =      NexPicture(1, 6, "Icon_Update");
      NexPicture nex_1_link =       NexPicture(1, 7, "Icon_Link");
      NexPicture nex_1_start =      NexPicture(1, 8, "Icon_Start");
    //Sys=2
      NexPicture nex_2_rs232 =      NexPicture(2, 6, "Icon_Update");
      NexPicture nex_2_link =       NexPicture(2, 7, "Icon_Link");
      NexPicture nex_2_start =      NexPicture(2, 8, "Icon_Start");
    //alarms=3
      NexPicture nex_3_rs232 =      NexPicture(3, 6, "Icon_Update");
      NexPicture nex_3_link =       NexPicture(3, 7, "Icon_Link");
      NexPicture nex_3_start =      NexPicture(3, 8, "Icon_Start");
    //sett_lock=4
      NexPicture nex_4_rs232 =      NexPicture(4, 6, "Icon_Update");
      NexPicture nex_4_link =       NexPicture(4, 7, "Icon_Link");
      NexPicture nex_4_start =      NexPicture(4, 8, "Icon_Start");
    //sett_lock=5
      NexPicture nex_5_rs232 =      NexPicture(5, 6, "Icon_Update");
      NexPicture nex_5_link =       NexPicture(5, 7, "Icon_Link");
      NexPicture nex_5_start =      NexPicture(5, 8, "Icon_Start");
    //pass_lock=6
      NexPicture nex_6_rs232 =      NexPicture(6, 6, "Icon_Update");
      NexPicture nex_6_link =       NexPicture(6, 7, "Icon_Link");
      NexPicture nex_6_start =      NexPicture(6, 8, "Icon_Start");
    //user_control=7
      NexPicture nex_7_rs232 =      NexPicture(7, 6, "Icon_Update");
      NexPicture nex_7_link =       NexPicture(7, 7, "Icon_Link");
      NexPicture nex_7_start =      NexPicture(7, 8, "Icon_Start");

  //Управление устройствами
    //Мешалки
      //М1
      NexButton nex_M1_mode =       NexButton(2, 37, "p27");
      NexButton nex_M1_start =      NexButton(2, 49, "p37");
      NexButton nex_M1_stop =       NexButton(2, 44, "p32");
      NexPicture nex_M1_stat =      NexPicture(2, 62, "p42");
      //М2
      NexButton nex_M2_mode =       NexButton(2, 38, "p28");
      NexButton nex_M2_start =      NexButton(2, 50, "p38");
      NexButton nex_M2_stop =       NexButton(2, 45, "p33");
      NexPicture nex_M2_stat =      NexPicture(2, 63, "p43");
      //М3
      NexButton nex_M3_mode =       NexButton(2, 39, "p29");
      NexButton nex_M3_start =      NexButton(2, 51, "p39");
      NexButton nex_M3_stop =       NexButton(2, 46, "p34");
      NexPicture nex_M3_stat =      NexPicture(2, 64, "p44");
    //Вибратор и клапан  
      //Вибратор на буфере
      NexButton nex_DS_mode =       NexButton(2, 40, "p30");
      NexButton nex_DS_start =      NexButton(2, 52, "p40");
      NexButton nex_DS_stop =       NexButton(2, 47, "p35");
      NexPicture nex_DS_stat =      NexPicture(2, 65, "p45");
      //Клапан подачи воды КС-1
      NexButton nex_KC_mode =       NexButton(2, 41, "p31");
      NexButton nex_KC_start =      NexButton(2, 53, "p41");
      NexButton nex_KC_stop =       NexButton(2, 48, "p36");
      NexPicture nex_KC_stat =      NexPicture(2, 66, "p46");
    //Опциональное оборудование  
      //Транспортный шнек
      NexButton nex_TH_mode =       NexButton(2, 60, "p13");
      NexButton nex_TH_start =      NexButton(2, 59, "p12");
      NexButton nex_TH_stop =       NexButton(2, 57, "p10");
      NexPicture nex_TH_stat =      NexPicture(2, 67, "p47");
      //Вибратор на бункере
      NexButton nex_TS_mode =       NexButton(2, 61, "p14");
      NexButton nex_TS_start =      NexButton(2, 58, "p11");
      NexButton nex_TS_stop =       NexButton(2, 56, "p9");
      NexPicture nex_TS_stat =      NexPicture(2, 68, "p48");
    //Шаговые привода
      //Дозирующий шнек
      NexText nex_SD_SP_imp =          NexText(7,26,"t11");
      NexText nex_SD_SP_ms =           NexText(7,27,"t12");
      NexButton nex_SD_cancel_sys =    NexButton(7,30,"p16");
      NexButton nex_SD_adapt =         NexButton(7,33,"p24");
      NexButton nex_SD_test =          NexButton(7,31,"p15");
      NexButton nex_SD_vector =        NexButton(7,35,"p49");

  //Датчики
      //Дискретные сигналы
        //низкий уровень в буфере
        NexButton nex_LLS_use =       NexButton(2, 19, "p18");
        NexButton nex_LLS_inv =       NexButton(2, 24, "p7");
        NexPicture nex_LLS_stat =     NexPicture(2, 21, "p2");
        //низкий уровень в буфере
        NexButton nex_HLS_use =       NexButton(2, 18, "p19");
        NexButton nex_HLS_inv =       NexButton(2, 25, "p17");
        NexPicture nex_HLS_stat =     NexPicture(2, 22, "p3");
        //низкий уровень в буфере
        NexButton nex_ALS_use =       NexButton(2, 17, "p20");
        NexButton nex_ALS_inv =       NexButton(2, 26, "p21");
        NexPicture nex_ALS_stat =     NexPicture(2, 23, "p6");
        //расходомер
        NexPicture nex_Flow_RM_stat =     NexPicture(2, 54, "p8");

  //Настройки
    //Количество секций
        NexButton nex_sec_2_use =     NexButton(2, 69, "p50");
        NexButton nex_sec_3_use =     NexButton(2, 71, "p51");
        NexButton nex_M2_speed =      NexButton(2, 74, "p52");
    //Настройки мешалок
        NexButton nex_M1_use  =       NexButton(1, 43, "p17");
        NexButton nex_M2_use  =       NexButton(1, 44, "p18");
        NexButton nex_M3_use  =       NexButton(1, 45, "p19");
        NexButton nex_M12_use  =      NexButton(1, 46, "p20");
        NexButton nex_M123_use  =     NexButton(1, 47, "p21");
        NexText nex_M1_SP =           NexText(1, 38, "t20");
        NexText nex_M2_SP =           NexText(1, 39, "t21");
        NexText nex_M3_SP =           NexText(1, 40, "t22");
        NexText nex_M2_Low_SP =       NexText(7, 41, "t15");
        NexText nex_M2_High_SP =      NexText(7, 42, "t16");
        NexText nex_KC_delay =        NexText(7, 45, "t20");
        NexText nex_T_TH_Limit =      NexText(7, 49, "t24");
    //Настройки вибраторов
        NexButton nex_DS_use =        NexButton(1, 52, "p22");
        NexButton nex_TH_use =        NexButton(1, 65, "p23");
        NexButton nex_TS_use =        NexButton(1, 69, "p24");
        NexText nex_DS_SP_work_ON =   NexText(1, 57, "t33");
        NexText nex_DS_SP_work_OFF =  NexText(1, 59, "t35");
        NexText nex_DS_SP_wait_ON =   NexText(1, 64, "t40");
        NexText nex_DS_SP_wait_OFF =  NexText(1, 62, "t38");
        NexText nex_TH_SP_ON =        NexText(1, 67, "t42");
        NexText nex_TS_SP_ON =        NexText(1, 71, "t45");
        NexText nex_TS_SP_OFF =       NexText(1, 73, "t47");
      //Настройка ModBus
        NexButton nex_ModBus_use =    NexButton(1, 75, "p25");
        NexText nex_ModBus_addr =     NexText(1, 78, "t51");
        NexPicture nex_ModBus_link =  NexPicture(1, 80, "p26");
      //Калибровка дозатора
        NexButton nex_dos_c_start =   NexButton(1, 27, "p15");
        NexButton nex_dos_c_cancel =  NexButton(1, 28, "p16");
        NexText nex_dos_c_distance =  NexText(1, 30, "t12");
        NexText nex_dos_c_speed =     NexText(1, 29, "t11");
        NexText nex_dos_c_mass =      NexText(1, 31, "t13");
        NexText nex_dos_c_source =    NexText(1, 49, "t26");

  //Общий вид
    //Статичные элементы
        NexPicture nex_sec_2 =        NexPicture(0, 33, "p3");
        NexPicture nex_sec_3 =        NexPicture(0, 34, "p4");

  //Главный экран
    //Устройства
        NexPicture nex_main_M1_status = NexPicture(0, 15, "p2");
        NexPicture nex_main_M2_status = NexPicture(0, 35, "p5");
        NexPicture nex_main_M3_status = NexPicture(0, 36, "p6");
        NexPicture nex_main_DS_status = NexPicture(0, 29, "p12");
        NexPicture nex_main_KC_status = NexPicture(0, 10, "img_kc");
        NexPicture nex_main_TH_status = NexPicture(0, 30, "p13");
        NexPicture nex_main_TS_status = NexPicture(0, 51, "p14");
        NexPicture nex_main_SD_status = NexPicture(0, 12, "img_dos");
        NexText nex_main_SD_speed =     NexText(0, 53, "t23");
    //Датчики
        NexPicture nex_main_LS =        NexPicture(0, 19, "p1");
        NexText nex_main_LE =           NexText(0, 28, "t14");
        NexProgressBar nex_main_PB =    NexProgressBar(0, 11, "j0");
        NexText nex_main_Flow =         NexText(0, 46, "t19");
    //Параметры
        NexText nex_main_SP_C =         NexText(0, 20, "t1");
        NexText nex_main_SP_High =      NexText(0, 22, "t6");
        NexText nex_main_SP_Low =       NexText(0, 25, "t11");
    //Управение
        NexButton nex_main_Start =      NexButton(0, 49, "p10");
        NexButton nex_main_Stop =       NexButton(0, 50, "p22");
        NexPicture nex_CMD_Save =       NexPicture(0, 54, "p15");
    //Прочее
        NexPicture nex_main_TH_pipe =   NexPicture(0, 38, "p11");
        NexPicture nex_main_TH_tube =   NexPicture(0, 37, "p9");
        NexText nex_main_M1_Name =      NexText(0, 40, "t3");
        NexText nex_main_M2_Name =      NexText(0, 41, "t4");
        NexText nex_main_M3_Name =      NexText(0, 42, "t8");
        NexText nex_main_TH_Name =      NexText(0, 31, "t15");
        NexText nex_main_DS_Name =      NexText(0, 32, "t16");

  //Экран аварий
    //Аварии устройств
        //QF
          NexPicture  nex_M1_alm_QF =     NexPicture(3, 31, "p2");
          NexPicture  nex_M2_alm_QF =     NexPicture(3, 32, "p3");
          NexPicture  nex_M3_alm_QF =     NexPicture(3, 33, "p6");
          NexPicture  nex_DS_alm_QF =     NexPicture(3, 34, "p7");
          NexPicture  nex_TH_alm_QF =     NexPicture(3, 35, "p8");
          NexPicture  nex_TS_alm_QF =     NexPicture(3, 36, "p9");
        //NoErr
          NexPicture  nex_M1_alm_NoErr =     NexPicture(3, 37, "p10");
          NexPicture  nex_M2_alm_NoErr =     NexPicture(3, 38, "p11");
          NexPicture  nex_M3_alm_NoErr =     NexPicture(3, 39, "p12");
          NexPicture  nex_DS_alm_NoErr =     NexPicture(3, 40, "p13");
          NexPicture  nex_TH_alm_NoErr =     NexPicture(3, 41, "p14");
          NexPicture  nex_TS_alm_NoErr =     NexPicture(3, 42, "p15");
        //Start
          NexPicture  nex_M1_alm_Start =     NexPicture(3, 43, "p16");
          NexPicture  nex_M2_alm_Start =     NexPicture(3, 44, "p17");
          NexPicture  nex_M3_alm_Start =     NexPicture(3, 45, "p18");
          NexPicture  nex_DS_alm_Start =     NexPicture(3, 46, "p19");
          NexPicture  nex_TH_alm_Start =     NexPicture(3, 47, "p20");
          NexPicture  nex_TS_alm_Start =     NexPicture(3, 48, "p21");
        //Start
          NexPicture  nex_M1_alm_Stop =     NexPicture(3, 49, "p24");
          NexPicture  nex_M2_alm_Stop =     NexPicture(3, 50, "p27");
          NexPicture  nex_M3_alm_Stop =     NexPicture(3, 51, "p28");
          NexPicture  nex_DS_alm_Stop =     NexPicture(3, 52, "p29");
          NexPicture  nex_TH_alm_Stop =     NexPicture(3, 53, "p30");
          NexPicture  nex_TS_alm_Stop =     NexPicture(3, 54, "p31");

    //Аварии заполнения
          //Индикация
            NexPicture nex_alm_x0 =   NexPicture(3, 61, "p32");
            NexPicture nex_alm_x1 =   NexPicture(3, 62, "p33");
            NexPicture nex_alm_x2 =   NexPicture(3, 63, "p34");
            NexPicture nex_alm_x3 =   NexPicture(3, 64, "p35");
            NexPicture nex_alm_x4 =   NexPicture(3, 65, "p36");
            NexPicture nex_alm_x5 =   NexPicture(3, 66, "p37");
          //Уставки
            NexText nex_alm_SP_x0 =   NexText(2, 86, "t28");
            NexText nex_alm_SP_x1 =   NexText(2, 89, "t31");
            NexText nex_alm_SP_x2 =   NexText(2, 90, "t32");
            NexText nex_alm_SP_x3 =   NexText(2, 93, "t35");
            NexText nex_alm_SP_x4 =   NexText(2, 92, "t34");
            NexText nex_alm_SP_x5 =   NexText(2, 96, "t38");
            NexText nex_alm_SP_LLS =  NexText(2, 98, "t40");
            NexText nex_alm_SP_HLS =  NexText(2, 99, "t41");

    //Аварии приготовления
          NexText nex_alm_dos_SP = NexText(3, 71, "t42");
          NexText nex_alm_dos_NOW= NexText(3, 70, "t41");
          NexPicture nex_alm_dos = NexPicture(3, 76, "p40");

    //Аварии - опции
          NexPicture nex_alm_ready = NexPicture(3, 75, "p39");
          NexPicture nex_alm_block = NexPicture(3, 74, "p38");
          NexButton nex_CMD_Restart = NexButton(3, 78, "p52");


  //АКТИВНЫЕ ЭЛЕМЕНТЫ НА ЭКРАНЕ
    NexTouch *nex_listen_list[] = {
      &nex_0_page,
      &nex_1_page,
      &nex_2_page,
      &nex_3_page,
      &nex_4_page,
      &nex_5_page,
      &nex_6_page,
      &nex_7_page,
    //Боковое меню
      &nex_0_menu_home,
      &nex_1_menu_home,
      &nex_2_menu_home,
      &nex_3_menu_home,
      &nex_4_menu_home,
      &nex_5_menu_home,
      &nex_6_menu_home,
      &nex_7_menu_home,
      &nex_0_menu_sett,
      &nex_1_menu_sett,
      &nex_2_menu_sett,
      &nex_3_menu_sett,
      &nex_4_menu_sett,
      &nex_5_menu_sett,
      &nex_6_menu_sett,
      &nex_7_menu_sett,
      &nex_0_menu_sys,
      &nex_1_menu_sys,
      &nex_2_menu_sys,
      &nex_3_menu_sys,
      &nex_4_menu_sys,
      &nex_5_menu_sys,
      &nex_6_menu_sys,
      &nex_7_menu_sys,
      &nex_0_menu_alarm,
      &nex_1_menu_alarm,
      &nex_2_menu_alarm,
      &nex_3_menu_alarm,
      &nex_4_menu_alarm,
      &nex_5_menu_alarm,
      &nex_6_menu_alarm,
      &nex_7_menu_alarm,
      &nex_0_menu_save,
      &nex_1_menu_save,
      &nex_2_menu_save,
      &nex_3_menu_save,
      &nex_4_menu_save,
      &nex_5_menu_save,
      &nex_6_menu_save,
      &nex_7_menu_save,
    //Авторизация
      &nex_3_login,
      &nex_4_login,
      &nex_5_login,
      &nex_6_login,
    //Исполнительные механизмы  
      &nex_M1_mode,
      &nex_M1_start,
      &nex_M1_stop,
      &nex_M2_mode,
      &nex_M2_start,
      &nex_M2_stop,
      &nex_M3_mode,
      &nex_M3_start,
      &nex_M3_stop,
      &nex_DS_mode,
      &nex_DS_start,
      &nex_DS_stop,
      &nex_KC_mode,
      &nex_KC_start,
      &nex_KC_stop,
      &nex_TH_mode,
      &nex_TH_start,
      &nex_TH_stop,
      &nex_TS_mode,
      &nex_TS_start,
      &nex_TS_stop,
    //Шнековый дозатор
      &nex_SD_test,
      &nex_SD_adapt,
      &nex_SD_vector,
      &nex_SD_cancel_sys,
    //Дискретные датчики
      &nex_LLS_use,
      &nex_LLS_inv,
      &nex_HLS_use,
      &nex_HLS_inv,
      &nex_ALS_use,
      &nex_ALS_inv,
    //Настройки
      &nex_sec_2_use,
      &nex_sec_3_use,
      &nex_M1_use,
      &nex_M2_use,
      &nex_M3_use,
      &nex_M12_use,
      &nex_M123_use,
      &nex_M2_speed,
      &nex_DS_use,
      &nex_TH_use,
      &nex_TS_use,
      &nex_ModBus_use,
      &nex_dos_c_start,
      &nex_dos_c_cancel,
    //Управление
      &nex_main_Start,
      &nex_main_Stop,
      &nex_CMD_Restart,
    //NULL  
      NULL
    };

  //ДЕЙСТВИЯ ДЛЯ АКТИВНЫХ ЭЛЕМЕНТОВ
    int value = 0;
    int load_page = 0;
    //Боковое меню
      void cb_menu_home(void *ptr)  {      if (load_page == 1) {nex_0_page.show(); scr = 0; load_page = 0;} }
      void cb_menu_sett(void *ptr)  {      value = read(nex_4_value); if ((value == 1 or sett_access == 1) and load_page == 1){ nex_1_page.show(); sett_access = 1; scr = 1; load_page = 0;}else{nex_4_page.show(); scr=4; load_page = 0;}   }
      void cb_menu_sys(void *ptr)   {      value = read(nex_5_value); if ((value == 1 or sys_access == 1) and load_page == 1){ nex_2_page.show(); sys_access = 1; scr = 2; load_page = 0;}else{nex_5_page.show(); scr=5; load_page = 0;}  }
      void cb_menu_alarm(void *ptr) {      if (load_page == 1) {nex_3_page.show(); scr = 3; load_page = 0;} }
      void cb_menu_pass(void *ptr)  {      if (load_page == 1) {nex_6_page.show(); scr = 6; load_page = 0;} }
      void cb_menu_user(void *ptr)  {      value = read(nex_6_value); if (value == 1 and load_page == 1){ nex_7_page.show(); scr = 7; load_page = 0;} }
      void cb_menu_save(void *ptr)  {      scr_update = 1;  Reset_All = 1;}
      void cb_update(void *ptr)     {      scr_update = 1;}
    //Исполнительные механизмы
      //Мешалка 1
      void cb_M1_mode(void *ptr)    { if (M1.Hand == 0) {M1.Hand = 1;} else {M1.Hand = 0;}  }
      void cb_M1_start(void *ptr)   { if (M1.Hand == 1) {M1.Start_M = 1;}                   }
      void cb_M1_stop(void *ptr)    { if (M1.Hand == 1) {M1.Start_M = 0;}                   }
      //Мешалка 2
      void cb_M2_mode(void *ptr)    { if (M2.Hand == 0) {M2.Hand = 1;} else {M2.Hand = 0;}  }
      void cb_M2_start(void *ptr)   { if (M2.Hand == 1) {M2.Start_M = 1;}                   }
      void cb_M2_stop(void *ptr)    { if (M2.Hand == 1) {M2.Start_M = 0;}                   }
      //Мешалка 3
      void cb_M3_mode(void *ptr)    { if (M3.Hand == 0) {M3.Hand = 1;} else {M3.Hand = 0;}  }
      void cb_M3_start(void *ptr)   { if (M3.Hand == 1) {M3.Start_M = 1;}                   }
      void cb_M3_stop(void *ptr)    { if (M3.Hand == 1) {M3.Start_M = 0;}                   }
      //Вибратор на буфере
      void cb_DS_mode(void *ptr)    { if (DS.Hand == 0) {DS.Hand = 1;} else {DS.Hand = 0;}  }
      void cb_DS_start(void *ptr)   { if (DS.Hand == 1) {DS.Start_M = 1;}                   }
      void cb_DS_stop(void *ptr)    { if (DS.Hand == 1) {DS.Start_M = 0;}                   }
      //Клапан подачи воды КС-1
      void cb_KC_mode(void *ptr)    { if (KC.Hand == 0) {KC.Hand = 1;} else {KC.Hand = 0;}  }
      void cb_KC_start(void *ptr)   { if (KC.Hand == 1) {KC.Start_M = 1;}                   }
      void cb_KC_stop(void *ptr)    { if (KC.Hand == 1) {KC.Start_M = 0;}                   }
      //Транспортный шнек
      void cb_TH_mode(void *ptr)    { if (TH.Hand == 0) {TH.Hand = 1;} else {TH.Hand = 0;}  }
      void cb_TH_start(void *ptr)   { if (TH.Hand == 1) {TH.Start_M = 1;}                   }
      void cb_TH_stop(void *ptr)    { if (TH.Hand == 1) {TH.Start_M = 0;}                   }
      //Вибратор на бункере
      void cb_TS_mode(void *ptr)    { if (TS.Hand == 0) {TS.Hand = 1;} else {TS.Hand = 0;}  }
      void cb_TS_start(void *ptr)   { if (TS.Hand == 1) {TS.Start_M = 1;}                   }
      void cb_TS_stop(void *ptr)    { if (TS.Hand == 1) {TS.Start_M = 0;}                   }
    //Шнековый дозатор
      void cb_SD_test(void *ptr)       { SD.Test = 1; SD.status_vis = SD.Status;}
      void cb_SD_cancel_sys(void *ptr) { SD.Cancel = 1;}
      void cb_SD_adapt(void *ptr)      { if (SD.Adapt == 0) {SD.Adapt = 1;} else {SD.Adapt = 0;}  }
      void cb_SD_vector(void *ptr)     { if (SD.Vector == 0) {SD.Vector = 1;} else {SD.Vector = 0;}  }
    //Дискретные датчики
      void cb_LLS_use(void *ptr)    { if (LLS.Use == 0) {LLS.Use = 1;} else {LLS.Use = 0;}  }
      void cb_LLS_inv(void *ptr)    { if (LLS.Inv == 0) {LLS.Inv = 1;} else {LLS.Inv = 0;}  }
      void cb_HLS_use(void *ptr)    { if (HLS.Use == 0) {HLS.Use = 1;} else {HLS.Use = 0;}  }
      void cb_HLS_inv(void *ptr)    { if (HLS.Inv == 0) {HLS.Inv = 1;} else {HLS.Inv = 0;}  } 
      void cb_ALS_use(void *ptr)    { if (ALS.Use == 0) {ALS.Use = 1;} else {ALS.Use = 0;}  }
      void cb_ALS_inv(void *ptr)    { if (ALS.Inv == 0) {ALS.Inv = 1;} else {ALS.Inv = 0;}  } 
    //Настройки
      //Количество секций
        void cb_sec_2_use(void *ptr)  { if (section_2 == 0) {section_2 = 1;} else {section_2 = 0;}  }   
        void cb_sec_3_use(void *ptr)  { if (section_3 == 0) {section_3 = 1;} else {section_3 = 0;}  }
      //Настройка мешалок
        void cb_M1_use(void *ptr)     { if (mix_1_use == 0) {mix_1_use = 1;} else {mix_1_use = 0;}  }
        void cb_M2_use(void *ptr)     { if (mix_2_use == 0) {mix_2_use = 1;} else {mix_2_use = 0;}  }
        void cb_M3_use(void *ptr)     { if (mix_3_use == 0) {mix_3_use = 1;} else {mix_3_use = 0;}  }
        void cb_M12_use(void *ptr)    { if (mix_12_use == 0) {mix_12_use = 1;} else {mix_12_use = 0;}  }
        void cb_M123_use(void *ptr)   { if (mix_123_use == 0) {mix_123_use = 1;} else {mix_123_use = 0;}  }
        void cb_M2_speed(void *ptr)   { if (mix_2_speed == 0) {mix_2_speed = 1;} else {mix_2_speed = 0;}  }
      //Настройки вибраторов
        void cb_DS_use(void *ptr)     { if (DS_use == 0) {DS_use = 1;} else {DS_use = 0;}  }
        void cb_TH_use(void *ptr)     { if (TH_use == 0) {TH_use = 1;} else {TH_use = 0;}  }
        void cb_TS_use(void *ptr)     { if (TS_use == 0) {TS_use = 1;} else {TS_use = 0;}  }
      //Настройка ModBus
        void cb_ModBus_use(void *ptr) { if (modbus_use == 0) {modbus_use = 1;} else {modbus_use = 0;}  }
      //Калибровка дозатора
        void cb_dos_c_start(void *ptr){ if (CMD_Start == 0) {SD.Calibration = 1;}}
        void cb_dos_c_cancel(void *ptr){SD.Cancel = 1;}
    //Управление
        void cb_main_start(void *ptr) {REQ_Start = 1;}
        void cb_main_stop(void *ptr)  {REQ_Start = 0;}
        void cb_restart(void *ptr)    { if (CMD_Restart == 0) {CMD_Restart = 1;} else {CMD_Restart =0;} ee_save_int(CMD_Restart, ee_CMD_Restart);}

//========> БАЗОВЫЙ ФУНКЦИОНАЛ
  //обработка таймеров
    // Cycles
      unsigned long timeSec;
      unsigned long timer1s;
      unsigned long timer05s;
      unsigned long timer01s;
      int pulse_1s  = 0;
      int pulse_05s = 0;
      int pulse_01s  = 0;
      void timersInit() 
      {
        unsigned long uptimeSec = millis() / 100;
        timeSec  = uptimeSec;
        timer1s  = uptimeSec;
        timer05s = uptimeSec;
        timer01s  = uptimeSec;
      }
      void drv_pulse() 
      {
        timeSec = millis() / 100;
        if (timeSec < timer1s)
        {
          timer1s = timeSec;
        }
        if (timeSec - timer1s  >=  10)  
        {
          timer1s  = timeSec; pulse_1s = 1;
        }
        if (timeSec < timer05s)
        {
          timer05s = timeSec;
        }
        if (timeSec - timer05s  >=  5)  
        {
          timer05s  = timeSec; pulse_05s = 1;
        }
        if (timeSec < timer01s)
        {
          timer01s = timeSec;
        }
        if (timeSec - timer01s  >=  1)  
        {
          timer01s  = timeSec; pulse_01s = 1;
        }

        //Вызов таймеров
          T_M1.exe(pulse_1s);
          T_M2.exe(pulse_1s);
          T_M3.exe(pulse_1s);
          T_DS_Work_ON.exe(pulse_01s);
          T_DS_Wait_ON.exe(pulse_01s);
          T_DS_Work_OFF.exe(pulse_01s);
          T_DS_Wait_OFF.exe(pulse_1s);
          T_LLS_delay.exe(pulse_01s);
          T_HLS_delay.exe(pulse_01s);
          T_TH_ON.exe(pulse_01s);
          T_TS_ON.exe(pulse_01s);
          T_TS_OFF.exe(pulse_1s);
          T_LLS_error.exe(pulse_1s);
          T_ModBus.exe(pulse_01s);
          T_KC_Delay.exe(pulse_01s);
          T_TH_Limit.exe(pulse_1s);

          T_Alm[0].exe(pulse_1s);
          T_Alm[1].exe(pulse_1s);
          T_Alm[2].exe(pulse_1s);
          T_Alm[3].exe(pulse_1s);
          T_Alm[4].exe(pulse_1s);
          T_Alm[5].exe(pulse_1s);

          T_Sett_Access.exe(pulse_1s);
          T_Sys_Access.exe(pulse_1s);

        //Контроль доступа  
          if (sett_access == 1) {T_Sett_Access.Start = 1;}
          if (sys_access == 1)  {T_Sys_Access.Start = 1;}
          if (T_Sett_Access.Exit == 1) {sett_access = 0; T_Sett_Access.Start = 0;}
          if (T_Sys_Access.Exit == 1)   {sys_access = 0; T_Sys_Access.Start = 0;}
      }
      void drv_end() 
      {
        pulse_1s = 0;
        pulse_05s = 0;
        pulse_01s = 0;
        //alarm_devices = 0;
        //any_alarm = 0;
        Reset_All = 0;
      }

  //АДРЕСАЦИЯ
    int DIx[30];
    int DOx[30];
    int AIx[10];
    int AOx[10];
    int x = 0;

    void config_new()
    {
      DIx[1] = 3;
      DIx[2] = 2;
      DIx[3] = 5;
      DIx[4] = 4;
      DIx[5] = 48;
      DIx[6] = 49;
      DIx[7] = 15;
      DIx[8] = 14;
      DIx[9] = 43;
      DIx[10] = 45;
      DIx[11] = 46;
      DIx[12] = 47;
      DIx[13] = 16;
      DIx[14] = 20;
      DIx[15] = 21;
      DIx[16] = 45;
      DIx[17] = 40;
      DIx[18] = 38;
      DIx[19] = 39;
      DIx[20] = 17;

      for (int i=1; i <= 20; i++)
        {
            pinMode(DIx[i], INPUT);
        }


      DOx[1] = 30;
      DOx[2] = 29;
      DOx[3] = 28;
      DOx[4] = 27;
      DOx[5] = 26;
      DOx[6] = 25;
      DOx[7] = 24;
      DOx[8] = 23;
      DOx[9] = 22;
      DOx[10] = 37;
      DOx[11] = 10;
      DOx[12] = 11;
      DOx[13] = 12;
      DOx[14] = 13;
      DOx[15] = 36;
      DOx[16] = 35;
      DOx[17] = 34;
      DOx[18] = 33;
      DOx[19] = 32;
      DOx[20] = 31;

      for (int i=1; i <= 20; i++)
      {
           pinMode(DOx[i], OUTPUT);
      }

      AIx[1] = 7;
      AIx[2] = 6;
      AIx[3] = 5;
      AIx[4] = 4;
      AIx[5] = 3;
      AIx[6] = 2;
      AIx[7] = 1;
      AIx[8] = 0;

      AOx[1] = 6;
      AOx[2] = 7;
      AOx[3] = 8;
      AOx[4] = 9;
    }

    void config_old()
    {
      DIx[1] = 43;
      DIx[2] = 45;
      DIx[3] = 46;
      DIx[4] = 47;
      DIx[5] = 48;
      DIx[6] = 49;
      DIx[7] = 15;
      DIx[8] = 14;
      DIx[9] = 3;
      DIx[10] = 2;
      DIx[11] = 4;
      DIx[12] = 4;
      DIx[13] = 15;
      DIx[14] = 14;
      DIx[15] = 13;
      DIx[16] = 41;
      DIx[17] = 40;
      DIx[18] = 38;
      DIx[19] = 39;
      DIx[20] = 17;

      for (int i=1; i <= 20; i++)
        {
            pinMode(DIx[i], INPUT);
        }

      DOx[1] = 36;
      DOx[2] = 13;
      DOx[3] = 12;
      DOx[4] = 11;
      DOx[5] = 10;
      DOx[6] = 30;
      DOx[7] = 31;
      DOx[8] = 32;
      DOx[9] = 33;
      DOx[10] = 34;
      DOx[11] = 35;
      DOx[12] = 25;
      DOx[13] = 26;
      DOx[14] = 27;
      DOx[15] = 2;
      DOx[16] = 29;
      DOx[17] = 37;
      DOx[18] = 22;
      DOx[19] = 23;
      DOx[20] = 24;

      for (int i=1; i <= 20; i++)
      {
           pinMode(DOx[i], OUTPUT);
      }

      AIx[1] = 7;
      AIx[2] = 6;
      AIx[3] = 5;
      AIx[4] = 4;
      AIx[5] = 3;
      AIx[6] = 2;
      AIx[7] = 1;
      AIx[8] = 0;

      AOx[1] = 6;
      AOx[2] = 7;
      AOx[3] = 8;
      AOx[4] = 9;
    }

  //SETUP
    void setup()
    {
      Timer1.initialize(50);
      Timer1.attachInterrupt(SD_exe);
      DDRH = B11111111;
      ee_loading();

      config_new();

      nexInit();

      //Конфигурация ModBus RTU
        slave.begin(115200);
        slave.setID(modbus_addr);

     //ВЫЗОВ ОБРАБОТЧИКОВ КНОПОК
        //Боковое меню
          nex_0_menu_home.attachPop   (cb_menu_home);
          nex_1_menu_home.attachPop   (cb_menu_home);
          nex_2_menu_home.attachPop   (cb_menu_home);
          nex_3_menu_home.attachPop   (cb_menu_home);
          nex_4_menu_home.attachPop   (cb_menu_home);
          nex_5_menu_home.attachPop   (cb_menu_home);
          nex_6_menu_home.attachPop   (cb_menu_home);
          nex_7_menu_home.attachPop   (cb_menu_home);

          nex_0_menu_sett.attachPop   (cb_menu_sett);
          nex_1_menu_sett.attachPop   (cb_menu_sett);
          nex_2_menu_sett.attachPop   (cb_menu_sett);
          nex_3_menu_sett.attachPop   (cb_menu_sett);
          nex_4_menu_sett.attachPop   (cb_menu_sett);
          nex_5_menu_sett.attachPop   (cb_menu_sett);
          nex_6_menu_sett.attachPop   (cb_menu_sett);
          nex_7_menu_sett.attachPop   (cb_menu_sett);

          nex_0_menu_sys.attachPop    (cb_menu_sys);
          nex_1_menu_sys.attachPop    (cb_menu_sys);
          nex_2_menu_sys.attachPop    (cb_menu_sys);
          nex_3_menu_sys.attachPop    (cb_menu_sys);
          nex_4_menu_sys.attachPop    (cb_menu_sys);
          nex_5_menu_sys.attachPop    (cb_menu_sys);
          nex_6_menu_sys.attachPop    (cb_menu_sys);
          nex_7_menu_sys.attachPop    (cb_menu_sys);

          nex_0_menu_alarm.attachPop  (cb_menu_alarm);
          nex_1_menu_alarm.attachPop  (cb_menu_alarm);
          nex_2_menu_alarm.attachPop  (cb_menu_alarm);
          nex_3_menu_alarm.attachPop  (cb_menu_alarm);
          nex_4_menu_alarm.attachPop  (cb_menu_alarm);
          nex_5_menu_alarm.attachPop  (cb_menu_alarm);
          nex_6_menu_alarm.attachPop  (cb_menu_alarm);
          nex_7_menu_alarm.attachPop  (cb_menu_alarm);

          nex_0_menu_save.attachPop   (cb_menu_save);
          nex_1_menu_save.attachPop   (cb_menu_save);
          nex_2_menu_save.attachPop   (cb_menu_save);
          nex_3_menu_save.attachPop   (cb_menu_save);
          nex_4_menu_save.attachPop   (cb_menu_save);
          nex_5_menu_save.attachPop   (cb_menu_save);
          nex_6_menu_save.attachPop   (cb_menu_save);
          nex_7_menu_save.attachPop   (cb_menu_save);

          nex_3_login.attachPop       (cb_menu_pass);
          nex_4_login.attachPop       (cb_menu_sett);
          nex_5_login.attachPop       (cb_menu_sys);
          nex_6_login.attachPop       (cb_menu_user);

        //Исполнительные механизмы
          nex_M1_mode.attachPop       (cb_M1_mode);
          nex_M1_start.attachPop      (cb_M1_start);
          nex_M1_stop.attachPop       (cb_M1_stop);
          nex_M2_mode.attachPop       (cb_M2_mode);
          nex_M2_start.attachPop      (cb_M2_start);
          nex_M2_stop.attachPop       (cb_M2_stop);
          nex_M3_mode.attachPop       (cb_M3_mode);
          nex_M3_start.attachPop      (cb_M3_start);
          nex_M3_stop.attachPop       (cb_M3_stop);
          nex_DS_mode.attachPop       (cb_DS_mode);
          nex_DS_start.attachPop      (cb_DS_start);
          nex_DS_stop.attachPop       (cb_DS_stop);
          nex_KC_mode.attachPop       (cb_KC_mode);
          nex_KC_start.attachPop      (cb_KC_start);
          nex_KC_stop.attachPop       (cb_KC_stop);
          nex_TH_mode.attachPop       (cb_TH_mode);
          nex_TH_start.attachPop      (cb_TH_start);
          nex_TH_stop.attachPop       (cb_TH_stop);
          nex_TS_mode.attachPop       (cb_TS_mode);
          nex_TS_start.attachPop      (cb_TS_start);
          nex_TS_stop.attachPop       (cb_TS_stop);
        //Дискретный датчики
          nex_LLS_use.attachPop       (cb_LLS_use);
          nex_LLS_inv.attachPop       (cb_LLS_inv);
          nex_HLS_use.attachPop       (cb_HLS_use);
          nex_HLS_inv.attachPop       (cb_HLS_inv);
          nex_ALS_use.attachPop       (cb_ALS_use);
          nex_ALS_inv.attachPop       (cb_ALS_inv);
        //Шнековый дозатор
          nex_SD_test.attachPop       (cb_SD_test);
          nex_SD_adapt.attachPop      (cb_SD_adapt);
          nex_SD_vector.attachPop     (cb_SD_vector);
          nex_SD_cancel_sys.attachPop (cb_SD_cancel_sys);
        //Настройки
          nex_sec_2_use.attachPop     (cb_sec_2_use);
          nex_sec_3_use.attachPop     (cb_sec_3_use);
          nex_M1_use.attachPop        (cb_M1_use);
          nex_M2_use.attachPop        (cb_M2_use);
          nex_M3_use.attachPop        (cb_M3_use);
          nex_M12_use.attachPop       (cb_M12_use);
          nex_M123_use.attachPop      (cb_M123_use);
          nex_M2_speed.attachPop      (cb_M2_speed);
          nex_DS_use.attachPop        (cb_DS_use);
          nex_TH_use.attachPop        (cb_TH_use);
          nex_TS_use.attachPop        (cb_TS_use);
          nex_ModBus_use.attachPop    (cb_ModBus_use);
          nex_dos_c_start.attachPop   (cb_dos_c_start);
          nex_dos_c_cancel.attachPop  (cb_dos_c_cancel);
        //Управление
          nex_main_Start.attachPop    (cb_main_start);
          nex_main_Stop.attachPop     (cb_main_stop);
          nex_CMD_Restart.attachPop   (cb_restart);

     //Конфигурация исполнительных механизмов
        //M1.config(DIx[1],DIx[11],0,DOx[1]);  //готовность, работа, авария, команда запуска <= 0 - не используется
        //M2.config(DIx[2],DIx[12],0,DOx[2]);  //готовность, работа, авария, команда запуска <= 0 - не используется
        //M3.config(DIx[3],DIx[13],0,DOx[3]);  //готовность, работа, авария, команда запуска <= 0 - не используется

        M1.config(DIx[1],DIx[2],0,DOx[1]);  //готовность, работа, авария, команда запуска <= 0 - не используется
        if (reagent == 0)
        {
          M2.config(DIx[3],DIx[4],0,DOx[2]);  //готовность, работа, авария, команда запуска <= 0 - не используется
          M3.config(DIx[5],DIx[6],0,DOx[3]);  //готовность, работа, авария, команда запуска <= 0 - не используется
          DS.config(DIx[7],0,0,DOx[5]);  //готовность, работа, авария, команда запуска <= 0 - не используется
          KC.config(0,0,0,DOx[11]);  //готовность, работа, авария, команда запуска <= 0 - не используется
        }
        if (reagent == 1)
        {
          M2.config(0,0,0,DOx[2]);  //готовность, работа, авария, команда запуска <= 0 - не используется
          M3.config(0,0,0,DOx[3]);  //готовность, работа, авария, команда запуска <= 0 - не используется
          DS.config(DIx[5],0,0,DOx[5]);  //готовность, работа, авария, команда запуска <= 0 - не используется
          KC.config(0,0,0,DOx[9]);  //готовность, работа, авария, команда запуска <= 0 - не используется
        }
        
        TH.config(0,0,0,DOx[4]);  //готовность, работа, авария, команда запуска <= 0 - не используется
        TS.config(0,0,0,DOx[6]);  //готовность, работа, авария, команда запуска <= 0 - не используется
        

      //Конфигурация дискретных датчиков
        if (reagent == 0)
        {
          LLS.config(DIx[11]);
          HLS.config(DIx[7]);
          ALS.config(DIx[8]);
          Flow_RM.config(DIx[10]);
          Start_REQ.config(DIx[12]);
          Block.config(DIx[9]);
        }
        if (reagent == 1)
        {
          LLS.config(DIx[3]);
          HLS.config(DIx[4]);
          ALS.config(DIx[5]);
          Flow_RM.config(DIx[6]);
          Start_REQ.config(DIx[7]);
          Block.config(DIx[8]);
        }

      //Конфигурация дискретных выходов
        LED_Start.config(DOx[17]);
        LED_Alarm.config(DOx[18]);
        LED_Dry_Run.config(DOx[19]);
        LED_Reagent.config(DOx[20]);

      //Конфигурация аналоговых датчиков
        LE.config(AIx[1], 1.0);

      //Конфигурация шаговых приводов
        SD.config(AOx[1],0,0);
      
      //Конфигурация таймеров
        timersInit();

      //КОНФИГУРАЦИЯ ПО УМОЛЧАНИЮ
        LLS.Use = 1;
        Flow_RM.Use = 1;
        DS_use = 1;
        T_LLS_error.tSP = 5;
        T_ModBus.tSP = 10;
        T_Sett_Access.tSP = 5;
        T_Sys_Access.tSP = 5;
        LE.LSx[5] = 1;
        if (mix_1_use == 1) {mixers++;}
        if (mix_2_use == 1) {mixers++;}
        if (mix_3_use == 1) {mixers++;}
        if (section_1 == 1) {sections++;}
        if (section_2 == 1) {sections++;}
        if (section_3 == 1) {sections++;}

        delay(2500);
        
        nex_0_page.show();

        //начало обмена данными
          nex_0_rs232.setPic(5);
          scr_cycle = 0;

        //уставки
          write(SP_C, nex_main_SP_C);
          write(SP_High, nex_main_SP_High);
          write(SP_Low, nex_main_SP_Low);

          nex_0_menu_save.Set_background_image_pic(29);

          HMI_Main();

          //уставки
                write(SP_C, nex_main_SP_C);
                write(LE.HS, nex_main_SP_High);
                write(LE.LS, nex_main_SP_Low);

        //конец обмена данными
          load_page = 1;
          nex_0_rs232.setPic(4);

        //WatchDog
          wdt_enable(WDTO_4S);
    }

  //Датчики
    void signals()
    {
      //Дискретные
        LLS.exe(pulse_01s, 5);
        HLS.exe(pulse_01s, 5);
        ALS.exe(pulse_01s, 5);
        Flow_RM.exe(pulse_01s, 0);
        Start_REQ.exe(pulse_1s, 0);
        Block.exe(pulse_01s, 0);

        RM.exe(KC.Status, Flow_RM.Status, pulse_1s);

        if (LLS.Use == 1 and HLS.Use == 1)
        {
          if (LLS.Status == 0 and HLS.Status == 0) {LS.Status = 0;}
          if (LLS.Status == 1 and HLS.Status == 0) {LS.Status = 1;}
          if (LLS.Status == 1 and HLS.Status == 1) {LS.Status = 2;}
          if (LLS.Status == 0 and HLS.Status == 1) {LS.Status = 3;}
        }

        if (LLS.Use == 1 and HLS.Use == 0)
        {
          if (LLS.Status == 0) {LS.Status = 0;}
          if (LLS.Status == 1) {LS.Status = 1;}
        }

        if (LLS.Use == 0 and HLS.Use == 0)
        {
          LS.Status = 0;
        }

      //Аналоговые
        LE.exe(1, Reset_All, pulse_01s);
    }

  //Исполнительные механизмы
    //Вызов на исполнение
      void devices()
      {
        M1.exe(pulse_1s, Reset_All);
        M2.exe(pulse_1s, Reset_All);
        M3.exe(pulse_1s, Reset_All);
        DS.exe(pulse_1s, Reset_All);
        KC.exe(pulse_1s, Reset_All);
        TH.exe(pulse_1s, Reset_All);
        TS.exe(pulse_1s, Reset_All);       

        //SD.exe();

        LED_Start.exe(CMD_Start);
        LED_Alarm.exe(any_alarm);
        LED_Dry_Run.exe(work_alarms[0]);
        LED_Reagent.exe(work_alarms[4]);
      }

//=========> NEXTION HMI
  //ИНДИКАТОРЫ
    //обшие индикаторы
      #define rs232_off 4
      #define rs232_on  5

      #define key_auto        44
      #define key_hand        45
      #define key_start_off   46
      #define key_start_on    47

      #define key_ON          14
      #define key_OFF         13

      #define key_select      10
      #define key_deselect    9

      #define status_off      15
      #define status_on       16
      #define status_wait     17
      #define status_error    18
      #define status_fault    83
      #define status_void     84

      #define CMD_Start_ON    50
      #define CMD_Start_OFF   49
      #define CMD_Start_Wait  85
      #define CMD_Save_ON     88
      #define CMD_Save_OFF    87
      #define CMD_Save_NO     73

    //картинки навигации
      #define img_home_active      19
      #define img_home             20
      #define img_sett_active      21
      #define img_sett             22
      #define img_sys_active       23
      #define img_sys              24
      #define img_not_alarm_active 25
      #define img_not_alarm        26
      #define img_alarm_active     27
      #define img_alarm            28

    //картинки статусов
      #define img_local       6
      #define img_link_ok     7
      #define img_link_error  8
      #define img_stop        2
      #define img_start       3
      #define img_wait        86
      #define img_save        29

    //шнековый дозатор
      #define dos_test_off    57
      #define dos_test_on     58
      #define dos_c_start_off 52
      #define dos_c_start_on  53

    //индикаторы домашнего экрана
      #define main_mix_off    31
      #define main_mix_on     32
      #define main_mix_wait   75
      #define main_mix_error  33
      #define main_mix_void   74
      #define main_kc_open    35
      #define main_kc_close   34
      #define main_xS_off     38
      #define main_xS_on      39
      #define main_xS_wait    72
      #define main_xS_error   71
      #define main_xS_void    73
      #define main_motor_off  36   
      #define main_motor_on   37
      #define main_motor_wait 78
      #define main_motor_error 76
      #define main_motor_void_m 77
      #define main_TH_void_t  79
      #define main_TH_void_p  82
      #define LS_Low          43
      #define LS_Normal       42
      #define LS_High         80
      #define LS_Error        81

    //очистка буфера текстового обмена с панелью
      char buffer[10] = {0};
      void clr()
      {
        memset(buffer, 0, sizeof(buffer));
      }

    //чтение текстовых данных
      float read(NexText _nex)
      {
        clr();
        _nex.getText(buffer, sizeof(buffer));       //тение текста
        return atof(buffer);                        //конвертация текста в число   
      }

    //запись текстовых данных
      void write(float _data, NexText _nex)
      {
        clr();
        dtostrf(_data,5,1,buffer); 
        _nex.setText(buffer);
      }
      void write2(float _data, NexText _nex)
      {
        clr();
        dtostrf(_data,5,2,buffer); 
        _nex.setText(buffer);
      }

    //Ручной режим
      int scr_dev_hand(DRV_MD device, NexButton nex_dev_hand, int update)
      {
        if (update == 0)
          { 
            if (device.Hand == 0) {nex_dev_hand.Set_background_image_pic(key_auto);}             
            else {nex_dev_hand.Set_background_image_pic(key_hand);} 
            return device.Hand;
          }
        else 
          {
            if (device.hand_vis != device.Hand)
            {
              if (device.Hand == 0) {nex_dev_hand.Set_background_image_pic(key_auto);}             
              else {nex_dev_hand.Set_background_image_pic(key_hand);} 
              return device.Hand;
            }
          }
      }

    //Управление в ручном режиме
      int scr_dev_start(DRV_MD device, NexButton nex_dev_start_m, int update)
      {
        if (update == 0)
          { 
            if (device.Start_M == 0)  {nex_dev_start_m.Set_background_image_pic(key_start_off);}   
            else {nex_dev_start_m.Set_background_image_pic(key_start_on);} 
            return device.Start_M;
          }
        else
        {
          if (device.start_vis != device.Start_M)
            {
                if (device.Start_M == 0)  {nex_dev_start_m.Set_background_image_pic(key_start_off); device.start_vis = 0;}   
                else {nex_dev_start_m.Set_background_image_pic(key_start_on); device.start_vis = 1;} 
                return device.Start_M;
            }
        } 
      }

    //Статус
      int status_list_sys[5] = {status_off, status_on, status_error, status_wait};
      int status_list_mix[5] = {main_mix_off, main_mix_on, main_mix_error, main_mix_wait, main_mix_void};
      int status_list_motor[5] = {main_motor_off, main_motor_on, main_motor_error, main_motor_wait, main_motor_void_m};
      int status_list_vibro[5] = {main_xS_off, main_xS_on, main_xS_error, main_xS_wait, main_xS_void};
      int status_list_LS[5] = {LS_Low, LS_Normal, LS_High, LS_Error, LS_Error};
      int status_list_KC[5] = {main_kc_close, main_kc_open};
      int status_list_CMD[5] = {CMD_Start_OFF, CMD_Start_ON, CMD_Start_Wait};
      int status_list_img_CMD[5] = {img_stop, img_start, img_wait, img_wait, img_wait};
      int status_list_CMD_save[5] = {CMD_Save_OFF, CMD_Save_ON, CMD_Save_NO, CMD_Save_NO, CMD_Save_NO};
      int status_list_modbus[5] = {img_local, img_link_ok, img_link_error, img_link_error, img_link_error};
      //привод
        int scr_dev_status(DRV_MD device, NexPicture nex_dev_status, int status[5], int use, int update)
        {
          if (update == 0)
          {
            if (use == 1)
            {
              if (device.Status == 0)   {nex_dev_status.setPic(status[0]);} 
              if (device.Status == 1)   {nex_dev_status.setPic(status[1]);} 
              if (device.Status >  3)   {nex_dev_status.setPic(status[2]);}
              if (device.Status > 1 and device.Status <= 3)  {nex_dev_status.setPic(status[3]);}
            }
            else
            {
              nex_dev_status.setPic(status[4]);
            }
            
            return device.Status;  
          }
          else
          {
            if (device.Status != device.status_vis)
            {
              if (use == 1)
              {
                if (device.Status == 0)   {nex_dev_status.setPic(status[0]);} 
                if (device.Status == 1)   {nex_dev_status.setPic(status[1]);} 
                if (device.Status >  3)   {nex_dev_status.setPic(status[2]);}
                if (device.Status > 1 and device.Status <= 3)  {nex_dev_status.setPic(status[3]);}
              }
              else
              {
                nex_dev_status.setPic(status[4]);
              }
              return device.Status; 
            }
          }
        }
      //ШД
        int scr_sd_status(DRV_SD device, NexPicture nex_dev_status, int status[5], int use, int update)
        {
          if (update == 0)
          {
              if (device.Status == 0)   {nex_dev_status.setPic(status[0]);} 
              if (device.Status == 1)   {nex_dev_status.setPic(status[1]);} 
              return device.Status;  
          }
          else
          {
            if (device.Status != device.status_vis)
            {
                if (device.Status == 0)   {nex_dev_status.setPic(status[0]);} 
                if (device.Status == 1)   {nex_dev_status.setPic(status[1]);} 
                return device.Status; 
            }
          }
        }
      //картинка
        int scr_any_status(int Status, int status_vis, NexPicture nex_any_status, int status[5], int use, int update)
        {
          if (update == 0)
          {
            if (use == 1)
            {
              if (Status == 0)   {nex_any_status.setPic(status[0]);} 
              if (Status == 1)   {nex_any_status.setPic(status[1]);} 
              if (Status == 2)   {nex_any_status.setPic(status[2]);}
              if (Status == 3)   {nex_any_status.setPic(status[3]);}
            }
            else
            {
              nex_any_status.setPic(status[4]);
            } 
          }
          else
          {
            if (Status != status_vis)
            {
              if (use == 1)
              {
                if (Status == 0)   {nex_any_status.setPic(status[0]);} 
                if (Status == 1)   {nex_any_status.setPic(status[1]);} 
                if (Status == 2)   {nex_any_status.setPic(status[2]);}
                if (Status == 3)   {nex_any_status.setPic(status[3]);}
              }
              else
              {
                nex_any_status.setPic(status[4]);
              }
            }
          }
          return Status; 
        }
      //кнопка
        int scr_key_status(int Status, int status_vis, NexButton nex_any_status, int status[5], int use, int update)
        {
          if (update == 0)
          {
            if (use == 1)
            {
              if (Status == 0)   {nex_any_status.Set_background_image_pic(status[0]);} 
              if (Status == 1)   {nex_any_status.Set_background_image_pic(status[1]);} 
              if (Status >  3)   {nex_any_status.Set_background_image_pic(status[2]);}
              if (Status > 1 and Status <= 3)  {nex_any_status.Set_background_image_pic(status[3]);}
            }
            else
            {
              nex_any_status.Set_background_image_pic(status[4]);
            }
            
            return Status;  
          }
          else
          {
            if (Status != status_vis)
            {
              if (use == 1)
              {
                if (Status == 0)   {nex_any_status.Set_background_image_pic(status[0]);} 
                if (Status == 1)   {nex_any_status.Set_background_image_pic(status[1]);} 
                if (Status >  3)   {nex_any_status.Set_background_image_pic(status[2]);}
                if (Status > 1 and Status <= 3)  {nex_any_status.Set_background_image_pic(status[3]);}
              }
              else
              {
                nex_any_status.Set_background_image_pic(status[4]);
              }
              return Status; 
            }
          }
        }

    //Переключаемый параметр
        int scr_use(int data, int data_vis, NexButton nex_use, int update)//, 
        {
          if (update == 0)
          {
            if (data == 0) {nex_use.Set_background_image_pic(key_OFF);}  else {nex_use.Set_background_image_pic(key_ON);}
            return data;
          }
          else
          {
            if (data != data_vis)
            {
              if (data == 0) {nex_use.Set_background_image_pic(key_OFF);}  else {nex_use.Set_background_image_pic(key_ON);}
              return data;
            }
          }
        }

    //Настройка дискретных сигналов - использовать
      int scr_xDI_use(DRV_DI xDI, NexButton nex_xDI_use, int update)//, 
      {
        if (update == 0)
        {
          if (xDI.Use == 0) {nex_xDI_use.Set_background_image_pic(key_OFF);}  else {nex_xDI_use.Set_background_image_pic(key_ON);}
          return xDI.Use;
        }
        else
        {
          if (xDI.Use != xDI.use_vis)
          {
            if (xDI.Use == 0) {nex_xDI_use.Set_background_image_pic(key_OFF);}  else {nex_xDI_use.Set_background_image_pic(key_ON);}
            return xDI.Use;
          }
        }
      }

    //Настройка дискретных сигналов - инверсия
      int scr_xDI_inv(DRV_DI xDI, NexButton nex_xDI_inv, int update)//, 
      {
        if (update == 0)
        {
          if (xDI.Inv == 0) {nex_xDI_inv.Set_background_image_pic(key_deselect);}  else {nex_xDI_inv.Set_background_image_pic(key_select);}
          return xDI.Inv;
        }
        else
        {
          if (xDI.Inv != xDI.inv_vis)
          {
            if (xDI.Inv == 0) {nex_xDI_inv.Set_background_image_pic(key_deselect);}  else {nex_xDI_inv.Set_background_image_pic(key_select);}
            return xDI.Inv;
          }
        }
      }
   
    //Отображение дискретного сигнала
      int scr_xDI_status(DRV_DI xDI, NexPicture nex_xDI_status, int OFF, int ON, int update)
      {
        if (update == 0)
        {
          if (xDI.Status == 0) {nex_xDI_status.Set_background_image_pic(OFF);}  else {nex_xDI_status.Set_background_image_pic(ON);}
          return xDI.Status;
        }
        else
        {
          if (xDI.Status != xDI.status_vis)
          {
            if (xDI.Status == 0) {nex_xDI_status.Set_background_image_pic(OFF);}  else {nex_xDI_status.Set_background_image_pic(ON);}
            return xDI.Status;
          }
        }
      }

    //Отображение произвольного состояния картинки
      int scr_status(int Status, int status_vis, NexPicture nex_status, int OFF, int ON, int update)
      {
        if (update == 0)
        {
          if (Status == 0) {nex_status.Set_background_image_pic(OFF);}  else {nex_status.Set_background_image_pic(ON);}
          return Status;
        }
        else
        {
          if (Status != status_vis)
          {
            if (Status == 0) {nex_status.Set_background_image_pic(OFF);}  else {nex_status.Set_background_image_pic(ON);}
            return Status;
          }
        }
      }

    //Отображение произвольного состояния кнопки
      int scr_status_key(int Status, int status_vis, NexButton nex_status, int OFF, int ON, int update)
      {
        if (update == 0)
        {
          if (Status == 0) {nex_status.Set_background_image_pic(OFF);}  else {nex_status.Set_background_image_pic(ON);}
          return Status;
        }
        else
        {
          if (Status != status_vis)
          {
            if (Status == 0) {nex_status.Set_background_image_pic(OFF);}  else {nex_status.Set_background_image_pic(ON);}
            return Status;
          }
        }
      }

  //Обновление данных при переходе на экран
      //Main = 0
        #define s_pic_0   59
        #define s_pic_1   60
        #define s_pic_2   61
        #define use_it  1

      //Main = 0
        void HMI_Main()
        {   
            any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_0_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
            modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_0_link, status_list_modbus, use_it, scr_cycle);
            CMD_Status_vis = scr_key_status(CMD_Status, CMD_Status_vis, nex_main_Start, status_list_CMD, use_it, scr_cycle);
            CMD_Save_vis = scr_any_status(CMD_Save, CMD_Save_vis, nex_CMD_Save, status_list_CMD_save, CMD_Restart, scr_cycle);

            CMD_0_vis = scr_any_status(CMD_Status, CMD_0_vis, nex_0_start, status_list_img_CMD, use_it, scr_cycle);
          //Отображение секций
            sec_2_vis = scr_status(section_2, sec_2_vis, nex_sec_2, s_pic_2, s_pic_1, scr_cycle);
            sec_3_vis = scr_status(section_3, sec_3_vis, nex_sec_3, s_pic_2, s_pic_1, scr_cycle);
          //Отображение приводов
            M1.status_vis = scr_dev_status(M1, nex_main_M1_status, status_list_mix, mix_1_use, scr_cycle);
            M2.status_vis = scr_dev_status(M2, nex_main_M2_status, status_list_mix, mix_2_use, scr_cycle);
            M3.status_vis = scr_dev_status(M3, nex_main_M3_status, status_list_mix, mix_3_use, scr_cycle);
            TH.status_vis = scr_dev_status(TH, nex_main_TH_status, status_list_motor, TH_use, scr_cycle);
            KC.status_vis = scr_dev_status(KC, nex_main_KC_status, status_list_KC, use_it, scr_cycle);
            SD.status_vis = scr_sd_status(SD, nex_main_SD_status, status_list_motor, use_it, scr_cycle);
          //Отображение вибраторов
            DS.status_vis = scr_dev_status(DS, nex_main_DS_status, status_list_vibro, DS_use, scr_cycle);
            TS.status_vis = scr_dev_status(TS, nex_main_TS_status, status_list_vibro, TS_use, scr_cycle);
          //Обработчик имён и скрываемых элементов
            if (scr_cycle == 0)
            {
              if (DS_use == 0)      {nex_main_DS_Name.setText(""); DS.status_vis = DS.Status;}
              if (TH_use == 0)      {nex_main_TH_Name.setText(""); nex_main_TH_tube.setPic(main_TH_void_t); nex_main_TH_pipe.setPic(main_TH_void_p); TH.status_vis = TH.Status;}
              if (mix_1_use == 0)   {nex_main_M1_Name.setText(""); M1.status_vis = M1.Status;}
              if (mix_2_use == 0)   {nex_main_M2_Name.setText(""); M2.status_vis = M2.Status;}
              if (mix_3_use == 0)   {nex_main_M3_Name.setText(""); M3.status_vis = M3.Status;}

              if (mix_1_use == 1)   {nex_main_M1_Name.setText("M1"); M1.status_vis = M1.Status;}
              if (mix_2_use == 1)   {nex_main_M2_Name.setText("M2"); M2.status_vis = M2.Status;}
              if (mix_3_use == 1)   {nex_main_M3_Name.setText("M3"); M3.status_vis = M3.Status;}
            }
            else
            {
              if (DS_use == 0 and DS.status_vis != DS.Status)      {nex_main_DS_Name.setText(""); DS.status_vis = DS.Status;}
              if (TH_use == 0 and TH.status_vis != TH.Status)      {nex_main_TH_Name.setText(""); TH.status_vis = TH.Status;}
              if (mix_1_use == 0 and M1.status_vis != M1.Status)   {nex_main_M1_Name.setText(""); M1.status_vis = M1.Status;}
              if (mix_2_use == 0 and M2.status_vis != M2.Status)   {nex_main_M2_Name.setText(""); M2.status_vis = M2.Status;}
              if (mix_3_use == 0 and M3.status_vis != M3.Status)   {nex_main_M3_Name.setText(""); M3.status_vis = M3.Status;}

              if (mix_1_use == 1 and M1.status_vis != M1.Status)   {nex_main_M1_Name.setText("M1"); M1.status_vis = M1.Status;}
              if (mix_2_use == 1 and M2.status_vis != M2.Status)   {nex_main_M2_Name.setText("M2"); M2.status_vis = M2.Status;}
              if (mix_3_use == 1 and M3.status_vis != M3.Status)   {nex_main_M3_Name.setText("M3"); M3.status_vis = M3.Status;}
            } 
          //Датчики
            LS.status_vis = scr_any_status(LS.Status, LS.status_vis, nex_main_LS, status_list_LS, use_it, scr_cycle);
            if (scr_cycle == 0 or (scr_cycle == 1 and LE.update() == 1)) 
            {
              write(LE.Result, nex_main_LE);
              nex_main_PB.setValue(LE.Result);
            }
            if (scr_cycle == 0 or (scr_cycle == 1 and RM.update() == 1)) 
            {
              write(RM.Flow, nex_main_Flow);
            }
            if (scr_cycle == 0 or (scr_cycle == 1 and SD.update() == 1)) 
            {
              write(SD.REQ_Speed, nex_main_SD_speed);
            }
        }

      //Sett = 1
        void HMI_Sett()
        {
            any_alarm_vis =     scr_status_key(any_alarm, any_alarm_vis, nex_1_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
            modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_1_link, status_list_modbus, use_it, scr_cycle);
            CMD_1_vis =         scr_any_status(CMD_Status, CMD_1_vis, nex_1_start, status_list_img_CMD, use_it, scr_cycle);
          //Настройки перемешивания
            mix_1_use_vis =     scr_use(mix_1_use, mix_1_use_vis, nex_M1_use, scr_cycle);
            mix_2_use_vis =     scr_use(mix_2_use, mix_2_use_vis, nex_M2_use, scr_cycle);
            mix_3_use_vis =     scr_use(mix_3_use, mix_3_use_vis, nex_M3_use, scr_cycle);
            mix_12_use_vis =    scr_use(mix_12_use, mix_12_use_vis, nex_M12_use, scr_cycle);
            mix_123_use_vis =   scr_use(mix_123_use, mix_123_use_vis, nex_M123_use, scr_cycle);
          //Настройки вибраторов
            DS_use_vis =        scr_use(DS_use, DS_use_vis, nex_DS_use, scr_cycle);
            TH_use_vis =        scr_use(TH_use, TH_use_vis, nex_TH_use, scr_cycle);
            TS_use_vis =        scr_use(TS_use, TS_use_vis, nex_TS_use, scr_cycle);
          //Настройка ModBus
            modbus_use_vis =    scr_use(modbus_use, modbus_use_vis, nex_ModBus_use, scr_cycle);
            error_link_vis =    scr_status(error_link, error_link_vis, nex_ModBus_link, status_on, status_error, scr_cycle);
          //Калибровка дозатора
            dos_c_start_vis =   scr_status_key(SD.Calibration, dos_c_start_vis, nex_dos_c_start, dos_c_start_off, dos_c_start_on, scr_cycle);
          
        }

      //Sys = 2
        void HMI_Sys()
        {
          
            any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_2_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
            modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_2_link, status_list_modbus, use_it, scr_cycle);
            CMD_2_vis = scr_any_status(CMD_Status, CMD_2_vis, nex_2_start, status_list_img_CMD, use_it, scr_cycle);
          //Исполнительные механизмы
            //Ручной режим
              M1.hand_vis = scr_dev_hand(M1, nex_M1_mode, scr_cycle);
              M2.hand_vis = scr_dev_hand(M2, nex_M2_mode, scr_cycle);
              M3.hand_vis = scr_dev_hand(M3, nex_M3_mode, scr_cycle);
              DS.hand_vis = scr_dev_hand(DS, nex_DS_mode, scr_cycle);
              KC.hand_vis = scr_dev_hand(KC, nex_KC_mode, scr_cycle);
              TH.hand_vis = scr_dev_hand(TH, nex_TH_mode, scr_cycle);
              TS.hand_vis = scr_dev_hand(TS, nex_TS_mode, scr_cycle);
            //Управление в ручном режиме
              M1.start_vis = scr_dev_start(M1, nex_M1_start, scr_cycle);
              M2.start_vis = scr_dev_start(M2, nex_M2_start, scr_cycle);
              M3.start_vis = scr_dev_start(M3, nex_M3_start, scr_cycle);
              DS.start_vis = scr_dev_start(DS, nex_DS_start, scr_cycle);
              KC.start_vis = scr_dev_start(KC, nex_KC_start, scr_cycle);
              TH.start_vis = scr_dev_start(TH, nex_TH_start, scr_cycle);
              TS.start_vis = scr_dev_start(TS, nex_TS_start, scr_cycle);  
            //Статус
              M1.status_vis = scr_dev_status(M1, nex_M1_stat, status_list_sys, use_it, scr_cycle);
              M2.status_vis = scr_dev_status(M2, nex_M2_stat, status_list_sys, use_it, scr_cycle);
              M3.status_vis = scr_dev_status(M3, nex_M3_stat, status_list_sys, use_it, scr_cycle);
              DS.status_vis = scr_dev_status(DS, nex_DS_stat, status_list_sys, use_it, scr_cycle);
              KC.status_vis = scr_dev_status(KC, nex_KC_stat, status_list_sys, use_it, scr_cycle);
              TH.status_vis = scr_dev_status(TH, nex_TH_stat, status_list_sys, use_it, scr_cycle);
              TS.status_vis = scr_dev_status(TS, nex_TS_stat, status_list_sys, use_it, scr_cycle);

          //Дискретные сигналы
            LLS.use_vis = scr_xDI_use(LLS, nex_LLS_use,scr_cycle);     LLS.inv_vis = scr_xDI_inv(LLS, nex_LLS_inv,scr_cycle);   LLS.status_vis = scr_xDI_status(LLS, nex_LLS_stat, status_off, status_on, scr_cycle);
            HLS.use_vis = scr_xDI_use(HLS, nex_HLS_use,scr_cycle);     HLS.inv_vis = scr_xDI_inv(HLS, nex_HLS_inv,scr_cycle);   HLS.status_vis = scr_xDI_status(HLS, nex_HLS_stat, status_off, status_on, scr_cycle);
            ALS.use_vis = scr_xDI_use(ALS, nex_ALS_use,scr_cycle);     ALS.inv_vis = scr_xDI_inv(ALS, nex_ALS_inv,scr_cycle);   ALS.status_vis = scr_xDI_status(ALS, nex_ALS_stat, status_off, status_on, scr_cycle);

            Flow_RM.status_vis = scr_xDI_status(Flow_RM, nex_Flow_RM_stat, status_off, status_on, scr_cycle);

          //Настройки
            mix_2_speed_vis =   scr_use(mix_2_speed, mix_2_speed_vis, nex_M2_speed, scr_cycle);
            sec_2_vis = scr_use(section_2, sec_2_vis, nex_sec_2_use, scr_cycle);
            sec_3_vis = scr_use(section_3, sec_3_vis, nex_sec_3_use, scr_cycle);

        }

      //Alarms = 3
        void HMI_Alm()
        {
          any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_3_menu_alarm, img_not_alarm_active, img_alarm_active, scr_cycle);
          modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_3_link, status_list_modbus, use_it, scr_cycle);
          CMD_3_vis = scr_any_status(CMD_Status, CMD_3_vis, nex_3_start, status_list_img_CMD, use_it, scr_cycle);

          int alm;
          //QF
            alm = 0;
            M1.alarms_vis[alm] = scr_status(M1.Alarms[alm], M1.alarms_vis[alm], nex_M1_alm_QF, status_on, status_fault, scr_cycle);
            M2.alarms_vis[alm] = scr_status(M2.Alarms[alm], M2.alarms_vis[alm], nex_M2_alm_QF, status_on, status_fault, scr_cycle);
            M3.alarms_vis[alm] = scr_status(M3.Alarms[alm], M3.alarms_vis[alm], nex_M3_alm_QF, status_on, status_fault, scr_cycle);
            DS.alarms_vis[alm] = scr_status(DS.Alarms[alm], DS.alarms_vis[alm], nex_DS_alm_QF, status_on, status_fault, scr_cycle);
            TH.alarms_vis[alm] = scr_status(TH.Alarms[alm], TH.alarms_vis[alm], nex_TH_alm_QF, status_on, status_fault, scr_cycle);
            TS.alarms_vis[alm] = scr_status(TS.Alarms[alm], TS.alarms_vis[alm], nex_TS_alm_QF, status_on, status_fault, scr_cycle);

          //NoErr
            alm = 1;
            M1.alarms_vis[alm] = scr_status(M1.Alarms[alm], M1.alarms_vis[alm], nex_M1_alm_NoErr, status_on, status_fault, scr_cycle);
            M2.alarms_vis[alm] = scr_status(M2.Alarms[alm], M2.alarms_vis[alm], nex_M2_alm_NoErr, status_on, status_fault, scr_cycle);
            M3.alarms_vis[alm] = scr_status(M3.Alarms[alm], M3.alarms_vis[alm], nex_M3_alm_NoErr, status_on, status_fault, scr_cycle);
            DS.alarms_vis[alm] = scr_status(DS.Alarms[alm], DS.alarms_vis[alm], nex_DS_alm_NoErr, status_on, status_fault, scr_cycle);
            TH.alarms_vis[alm] = scr_status(TH.Alarms[alm], TH.alarms_vis[alm], nex_TH_alm_NoErr, status_on, status_fault, scr_cycle);
            TS.alarms_vis[alm] = scr_status(TS.Alarms[alm], TS.alarms_vis[alm], nex_TS_alm_NoErr, status_on, status_fault, scr_cycle);

          //Start
            alm = 2;
            M1.alarms_vis[alm] = scr_status(M1.Alarms[alm], M1.alarms_vis[alm], nex_M1_alm_Start, status_void, status_fault, scr_cycle);
            M2.alarms_vis[alm] = scr_status(M2.Alarms[alm], M2.alarms_vis[alm], nex_M2_alm_Start, status_void, status_fault, scr_cycle);
            M3.alarms_vis[alm] = scr_status(M3.Alarms[alm], M3.alarms_vis[alm], nex_M3_alm_Start, status_void, status_fault, scr_cycle);
            DS.alarms_vis[alm] = scr_status(DS.Alarms[alm], DS.alarms_vis[alm], nex_DS_alm_Start, status_void, status_fault, scr_cycle);
            TH.alarms_vis[alm] = scr_status(TH.Alarms[alm], TH.alarms_vis[alm], nex_TH_alm_Start, status_void, status_fault, scr_cycle);
            TS.alarms_vis[alm] = scr_status(TS.Alarms[alm], TS.alarms_vis[alm], nex_TS_alm_Start, status_void, status_fault, scr_cycle);

          //Stop
            alm = 3;
            M1.alarms_vis[alm] = scr_status(M1.Alarms[alm], M1.alarms_vis[alm], nex_M1_alm_Stop, status_void, status_fault, scr_cycle);
            M2.alarms_vis[alm] = scr_status(M2.Alarms[alm], M2.alarms_vis[alm], nex_M2_alm_Stop, status_void, status_fault, scr_cycle);
            M3.alarms_vis[alm] = scr_status(M3.Alarms[alm], M3.alarms_vis[alm], nex_M3_alm_Stop, status_void, status_fault, scr_cycle);
            DS.alarms_vis[alm] = scr_status(DS.Alarms[alm], DS.alarms_vis[alm], nex_DS_alm_Stop, status_void, status_fault, scr_cycle);
            TH.alarms_vis[alm] = scr_status(TH.Alarms[alm], TH.alarms_vis[alm], nex_TH_alm_Stop, status_void, status_fault, scr_cycle);
            TS.alarms_vis[alm] = scr_status(TS.Alarms[alm], TS.alarms_vis[alm], nex_TS_alm_Stop, status_void, status_fault, scr_cycle);

          //Приготовление
            //0 - сухой ход, 1 - перелив, 2 - нет протока, 3 - нет заполнения, 4 - буфер пуст, 5 - бункер пуст, 6 - дозация, 7 - разрешение, 8 - внешняя блокировка
            work_alarms_vis[0] = scr_status(work_alarms[0], work_alarms_vis[0], nex_alm_x0, status_void, status_fault, scr_cycle);
            work_alarms_vis[1] = scr_status(work_alarms[1], work_alarms_vis[1], nex_alm_x1, status_void, status_fault, scr_cycle);
            work_alarms_vis[2] = scr_status(work_alarms[2], work_alarms_vis[2], nex_alm_x2, status_void, status_fault, scr_cycle);
            work_alarms_vis[3] = scr_status(work_alarms[3], work_alarms_vis[3], nex_alm_x3, status_void, status_fault, scr_cycle);
            work_alarms_vis[4] = scr_status(work_alarms[4], work_alarms_vis[4], nex_alm_x4, status_void, status_fault, scr_cycle);
            work_alarms_vis[5] = scr_status(work_alarms[5], work_alarms_vis[5], nex_alm_x5, status_void, status_fault, scr_cycle);
            work_alarms_vis[6] = scr_status(work_alarms[6], work_alarms_vis[6], nex_alm_dos, status_void, status_fault, scr_cycle);

            work_alarms_vis[8] = scr_status(work_alarms[8], work_alarms_vis[8], nex_alm_ready, status_fault, status_on, scr_cycle);
            work_alarms_vis[7] = scr_status(work_alarms[7], work_alarms_vis[7], nex_alm_block, status_on, status_fault, scr_cycle);

            CMD_Restart_vis = scr_status_key(CMD_Restart, CMD_Restart_vis, nex_CMD_Restart, key_OFF, key_ON, scr_cycle);

        }

      //sett_lock = 4
        void HMI_Sett_Lock()
        {
            any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_4_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
            modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_4_link, status_list_modbus, use_it, scr_cycle);
            CMD_4_vis = scr_any_status(CMD_Status, CMD_4_vis, nex_4_start, status_list_img_CMD, use_it, scr_cycle);
        }
      
      //sys_lock = 5
        void HMI_Sys_Lock()
        {
            any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_5_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
            modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_5_link, status_list_modbus, use_it, scr_cycle);
            CMD_5_vis = scr_any_status(CMD_Status, CMD_5_vis, nex_5_start, status_list_img_CMD, use_it, scr_cycle);
        }

      //pass_lock = 6
        void HMI_Pass_Lock()
        {
            any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_6_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
            modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_6_link, status_list_modbus, use_it, scr_cycle);
            CMD_6_vis = scr_any_status(CMD_Status, CMD_6_vis, nex_6_start, status_list_img_CMD, use_it, scr_cycle);
        }

      //UserControl = 7
        void HMI_User_Control()
        {
            any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_7_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
            modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_7_link, status_list_modbus, use_it, scr_cycle);
            CMD_7_vis = scr_any_status(CMD_Status, CMD_7_vis, nex_7_start, status_list_img_CMD, use_it, scr_cycle);

          //Шнековый дозатор
            SD.test_vis = scr_status_key(SD.Test, SD.test_vis, nex_SD_test, dos_test_off, dos_test_on, scr_cycle);
            SD.adapt_vis = scr_status_key(SD.Adapt, SD.adapt_vis, nex_SD_adapt, key_OFF, key_ON, scr_cycle);
            SD.vector_vis = scr_status_key(SD.Vector, SD.vector_vis, nex_SD_vector, key_OFF, key_ON, scr_cycle);
        }
  //Навигация
    void HMI_Menu()
    { 
      //Навигация
        if (scr != scr_active)
        {

          //Main=0
            if(scr == 0)
              {
                //начало обмена данными
                nex_0_rs232.setPic(rs232_on);
                scr_cycle = 0;
                nex_0_menu_home.Set_background_image_pic(img_home_active);
                nex_0_menu_sett.Set_background_image_pic(img_sett);
                nex_0_menu_sys.Set_background_image_pic(img_sys);
                nex_0_menu_save.Set_background_image_pic(img_save);
                any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_0_menu_alarm, img_not_alarm, img_alarm, scr_cycle); 
                modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_0_link, status_list_modbus, use_it, scr_cycle);

                //уставки
                write2(SP_C, nex_main_SP_C);
                write(LE.HS, nex_main_SP_High);
                write(LE.LS, nex_main_SP_Low);

                HMI_Main();

                //конец обмена данными
                nex_0_rs232.setPic(rs232_off);
              }

          //Sett=1
            if(scr == 1)
              {
                //начало обмена данными
                nex_1_rs232.setPic(rs232_on);
                scr_cycle = 0;
                nex_1_menu_home.Set_background_image_pic(img_home);
                nex_1_menu_sett.Set_background_image_pic(img_sett_active);
                nex_1_menu_sys.Set_background_image_pic(img_sys);
                nex_1_menu_save.Set_background_image_pic(img_save); 
                any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_1_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
                modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_1_link, status_list_modbus, use_it, scr_cycle);

                //время работы мешалок
                write(T_M1.tSP, nex_M1_SP);
                write(T_M2.tSP, nex_M2_SP);
                write(T_M3.tSP, nex_M3_SP);
                //время работы вибратора на буфере
                write(T_DS_Work_ON.tSP, nex_DS_SP_work_ON);
                write(T_DS_Work_OFF.tSP, nex_DS_SP_work_OFF);
                write(T_DS_Wait_ON.tSP, nex_DS_SP_wait_ON);
                write(T_DS_Wait_OFF.tSP, nex_DS_SP_wait_OFF);
                //время работы транспортного шнека
                write(T_TH_ON.tSP, nex_TH_SP_ON);
                //время работы вибратора на бункере
                write(T_TS_ON.tSP, nex_TS_SP_ON);
                write(T_TS_OFF.tSP, nex_TS_SP_OFF);
                //адрес ModBus
                write(modbus_addr, nex_ModBus_addr);
                //Калибровка дозатора
                write(dos_c_distance, nex_dos_c_distance);
                write(dos_c_speed, nex_dos_c_speed);
                write(dos_c_mass, nex_dos_c_mass);
                write(dos_c_source, nex_dos_c_source);

                HMI_Sett();               

                //конец обмена данными
                nex_1_rs232.setPic(rs232_off);
              }

          //Sys=2
            if(scr == 2)
              {
                //начало обмена данными
                nex_2_rs232.setPic(rs232_on);
                scr_cycle = 0;
                nex_2_menu_home.Set_background_image_pic(img_home);
                nex_2_menu_sett.Set_background_image_pic(img_sett);
                nex_2_menu_sys.Set_background_image_pic(img_sys_active); 
                nex_2_menu_save.Set_background_image_pic(img_save);
                any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_2_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
                modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_2_link, status_list_modbus, use_it, scr_cycle);

                write(T_Alm[0].tSP, nex_alm_SP_x0);
                write(T_Alm[1].tSP, nex_alm_SP_x1);
                write(T_Alm[2].tSP, nex_alm_SP_x2);
                write(T_Alm[3].tSP, nex_alm_SP_x3);
                write(T_Alm[4].tSP, nex_alm_SP_x4);
                write(T_Alm[5].tSP, nex_alm_SP_x5);

                write(LE.LLS, nex_alm_SP_LLS);
                write(LE.HHS, nex_alm_SP_HLS);

                HMI_Sys();

                //конец обмена данными
                nex_2_rs232.setPic(rs232_off);
              }

          //alarm=3
            if(scr == 3)
              {
                //начало обмена данными
                nex_3_rs232.setPic(rs232_on);
                scr_cycle = 0;
                nex_3_menu_home.Set_background_image_pic(img_home);
                nex_3_menu_sett.Set_background_image_pic(img_sett);
                nex_3_menu_sys.Set_background_image_pic(img_sys); 
                nex_3_menu_save.Set_background_image_pic(img_save);
                any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_3_menu_alarm, img_not_alarm_active, img_alarm_active, scr_cycle);
                modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_3_link, status_list_modbus, use_it, scr_cycle);

                HMI_Alm();

                write(dos_cnt, nex_alm_dos_NOW);
                write(dos_cnt_max, nex_alm_dos_SP);
                
                //конец обмена данными
                nex_3_rs232.setPic(rs232_off);
              }      
     
          //sett_lock = 4
            if (scr == 4)
            {
              //начало обмена данными
                nex_4_rs232.setPic(rs232_on);
                scr_cycle = 0;
                nex_4_menu_home.Set_background_image_pic(img_home);
                nex_4_menu_sett.Set_background_image_pic(img_sett_active);
                nex_4_menu_sys.Set_background_image_pic(img_sys); 
                nex_4_menu_save.Set_background_image_pic(img_save);
                any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_4_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
                modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_4_link, status_list_modbus, use_it, scr_cycle);

              //конец обмена данными
                nex_4_rs232.setPic(rs232_off);
            }

          //sys_lock = 5
            if (scr == 5)
            {
              //начало обмена данными
                nex_5_rs232.setPic(rs232_on);
                scr_cycle = 0;
                nex_5_menu_home.Set_background_image_pic(img_home);
                nex_5_menu_sett.Set_background_image_pic(img_sett);
                nex_5_menu_sys.Set_background_image_pic(img_sys_active); 
                nex_5_menu_save.Set_background_image_pic(img_save);
                any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_5_menu_alarm, img_not_alarm, img_alarm, scr_cycle);
                modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_5_link, status_list_modbus, use_it, scr_cycle);

              //конец обмена данными
                nex_5_rs232.setPic(rs232_off);
            }

          //pass_lock = 6
            if (scr == 6)
            {
              //начало обмена данными
                nex_6_rs232.setPic(rs232_on);
                scr_cycle = 0;
                nex_6_menu_home.Set_background_image_pic(img_home);
                nex_6_menu_sett.Set_background_image_pic(img_sett);
                nex_6_menu_sys.Set_background_image_pic(img_sys); 
                nex_6_menu_save.Set_background_image_pic(img_save);
                any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_6_menu_alarm, img_not_alarm_active, img_alarm_active, scr_cycle);
                modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_6_link, status_list_modbus, use_it, scr_cycle);

              //конец обмена данными
                nex_6_rs232.setPic(rs232_off);
            }

          //user_control = 7
            if (scr == 7)
            {
              //начало обмена данными
                nex_7_rs232.setPic(rs232_on);
                scr_cycle = 0;
                nex_7_menu_home.Set_background_image_pic(img_home);
                nex_7_menu_sett.Set_background_image_pic(img_sett);
                nex_7_menu_sys.Set_background_image_pic(img_sys); 
                nex_7_menu_save.Set_background_image_pic(img_save);
                any_alarm_vis = scr_status_key(any_alarm, any_alarm_vis, nex_7_menu_alarm, img_not_alarm_active, img_alarm_active, scr_cycle);
                modbus_status_vis = scr_any_status(modbus_status, modbus_status_vis, nex_7_link, status_list_modbus, use_it, scr_cycle);

                write(SD.SP_imp, nex_SD_SP_imp);
                write(SD.SP_ms, nex_SD_SP_ms);
                write(M2_Low_SP, nex_M2_Low_SP);
                write(M2_High_SP, nex_M2_High_SP);
                write(T_KC_Delay.tSP, nex_KC_delay);
                write(T_TH_Limit.tSP, nex_T_TH_Limit);

              //конец обмена данными
                nex_7_rs232.setPic(rs232_off);
            }
          //бокое меню обновлено
          scr_active = scr;
          load_page = 1;
        }
    }

  //Фоновое обновление
    void HMI_Update()
    {
      //Фоновое обновление экранов
        //Main = 0
          if (scr_active == 0 and pulse_1s == 1)
          {
            scr_cycle = 1;
            //write(LE.Result, nex_main_LE);
            //nex_main_PB.setValue(LE.Result);
            HMI_Main();
          }
        //Sett = 1
          if (scr_active == 1 and pulse_1s == 1)
          {
            scr_cycle = 1;
            HMI_Sett();
          }
        //Sys = 2
          if (scr_active == 2 and pulse_1s == 1)
          {
            scr_cycle = 1;
            HMI_Sys();
          }
        //alarm = 3
          if (scr_active == 3 and pulse_1s == 1)
          {
            scr_cycle = 1;
            HMI_Alm();
            write(dos_cnt, nex_alm_dos_NOW);
          }
        //sett_lock = 4
          if (scr_active == 4 and pulse_1s == 1)
          {
            scr_cycle = 1;
            HMI_Sett_Lock();
          }
        //sys_lock = 5
          if (scr_active == 5 and pulse_1s == 1)
          {
            scr_cycle = 1;
            HMI_Sys_Lock();
          }
        //pass_lock = 6
          if (scr_active == 6 and pulse_1s == 1)
          {
            scr_cycle = 1;
            HMI_Pass_Lock();
          }
        //user_control = 7
          if (scr_active == 7 and pulse_1s == 1)
          {
            scr_cycle = 1;
            HMI_User_Control();
          }
      }

  //Сохранение данных
      void HMI_Save()
      {
      //Сохранение данных
        //Main = 0
          if (scr_active == 0 and scr_update == 1)
          {
            //начало обмена данными
            nex_0_rs232.setPic(rs232_on);

            SP_C = read(nex_main_SP_C);
            LE.HS = read(nex_main_SP_High);
            LE.LS = read(nex_main_SP_Low);

            scr_cycle = 0;
            HMI_Main();

            nex_0_menu_save.Set_background_image_pic(img_save);

            //конец обмена данными
            nex_0_rs232.setPic(rs232_off);
            ee_saving();
            scr_update = 0;
          }
          
        //Sett = 1
          if (scr_active == 1 and scr_update == 1)
          {
            //начало обмена данными
            nex_1_rs232.setPic(rs232_on);

            //время работы мешалок
            T_M1.tSP = read(nex_M1_SP);
            T_M2.tSP = read(nex_M2_SP);
            T_M3.tSP = read(nex_M3_SP);
            //время работы вибратора на буфере
            T_DS_Work_ON.tSP = read(nex_DS_SP_work_ON);
            T_DS_Work_OFF.tSP = read(nex_DS_SP_work_OFF);
            T_DS_Wait_ON.tSP = read(nex_DS_SP_wait_ON);
            T_DS_Wait_OFF.tSP = read(nex_DS_SP_wait_OFF);
            //время работы транспортного шнека
            T_TH_ON.tSP = read(nex_TH_SP_ON);
            //время работы вибратора на бункере
            T_TS_ON.tSP = read(nex_TS_SP_ON);
            T_TS_OFF.tSP = read(nex_TS_SP_OFF);
            //адрес ModBus
            modbus_addr = int(read(nex_ModBus_addr)); 
            //Калибровка дозатора
            dos_c_distance = read(nex_dos_c_distance);
            dos_c_speed = read(nex_dos_c_speed);
            dos_c_mass = read(nex_dos_c_mass);
            dos_c_source = read(nex_dos_c_source);

            scr_cycle = 0;
            HMI_Sett();

            nex_1_menu_save.Set_background_image_pic(img_save);

            //конец обмена данными
            nex_1_rs232.setPic(rs232_off);
            ee_saving();
            scr_update = 0;
          }

        //Sys = 2
          if (scr_active == 2 and scr_update == 1)
          {
            //начало обмена данными
            nex_2_rs232.setPic(rs232_on);

            T_Alm[0].tSP = read(nex_alm_SP_x0);
            T_Alm[1].tSP = read(nex_alm_SP_x1);
            T_Alm[2].tSP = read(nex_alm_SP_x2);
            T_Alm[3].tSP = read(nex_alm_SP_x3);
            T_Alm[4].tSP = read(nex_alm_SP_x4);
            T_Alm[5].tSP = read(nex_alm_SP_x5);

            LE.LLS = read(nex_alm_SP_LLS);
            LE.HHS = read(nex_alm_SP_HLS);

            scr_cycle = 0;
            HMI_Sys();

            nex_2_menu_save.Set_background_image_pic(img_save);

            //конец обмена данными
            nex_2_rs232.setPic(rs232_off);
            ee_saving();
            scr_update = 0;
          }

        //alarm = 3
          if (scr_active == 3 and scr_update == 1)
          {
            //начало обмена данными
            nex_3_rs232.setPic(rs232_on);

            dos_cnt_max = read(nex_alm_dos_SP);

            nex_3_menu_save.Set_background_image_pic(img_save);

            write(dos_cnt, nex_alm_dos_NOW);

            scr_cycle = 0;
            HMI_Alm();

            //конец обмена данными
            nex_3_rs232.setPic(rs232_off);
            ee_saving();
            scr_update = 0;
          }

        //sett_lock = 4
          if (scr_active == 4 and scr_update == 1)
          {
            //начало обмена данными
            nex_4_rs232.setPic(rs232_on);

            scr_cycle = 0;
            HMI_Sett_Lock();

            nex_4_menu_save.Set_background_image_pic(img_save);

            //конец обмена данными
            nex_4_rs232.setPic(rs232_off);
            ee_saving();
            scr_update = 0;
          }

        //sys_lock = 5
          if (scr_active == 5 and scr_update == 1)
          {
            //начало обмена данными
            nex_5_rs232.setPic(rs232_on);

            scr_cycle = 0;
            HMI_Sys_Lock();

            nex_5_menu_save.Set_background_image_pic(img_save);

            //конец обмена данными
            nex_5_rs232.setPic(rs232_off);
            ee_saving();
            scr_update = 0;
          }

        //pass_lock = 6
          if (scr_active == 6 and scr_update == 1)
          {
            //начало обмена данными
            nex_6_rs232.setPic(rs232_on);

            scr_cycle = 0;
            HMI_Pass_Lock();

            nex_6_menu_save.Set_background_image_pic(img_save);

            //конец обмена данными
            nex_6_rs232.setPic(rs232_off);
            ee_saving();
            scr_update = 0;
          }

        //user_control = 7
          if (scr_active == 7 and scr_update == 1)
          {
            //начало обмена данными
            nex_7_rs232.setPic(rs232_on);

            SD.SP_imp = read(nex_SD_SP_imp);
            SD.SP_ms = read(nex_SD_SP_ms);
            M2_Low_SP = read(nex_M2_Low_SP);
            M2_High_SP = read(nex_M2_High_SP);
            T_KC_Delay.tSP = read(nex_KC_delay);
            T_TH_Limit.tSP = read(nex_T_TH_Limit);

            scr_cycle = 0;
            HMI_User_Control();

            nex_7_menu_save.Set_background_image_pic(img_save);

            //конец обмена данными
            nex_7_rs232.setPic(rs232_off);
            ee_saving();
            scr_update = 0;
          }
      }

  //HMI
      void HMI()
      {
        HMI_Menu();
        HMI_Update();
        HMI_Save();
      }

//==========> АЛГОРИТМИКА
    void PolyMix()
    {
      if (REQ_Start == 1 and Start_REQ.Status == 1) { CMD_Status = 1; CMD_Start = 1;}
      if (REQ_Start == 0 and Start_REQ.Status == 1) { CMD_Status = 0; CMD_Start = 0;}
      if (REQ_Start == 0 and Start_REQ.Status == 0) { CMD_Status = 0; CMD_Start = 0;}
      if (REQ_Start == 1 and Start_REQ.Status == 0) { CMD_Status = 5; CMD_Start = 0;}

      if (Block.Status == 0)  {REQ_Start = 0;}

      if (modbus_use == 0)                      {modbus_status = 0;}
      if (modbus_use == 1 and error_link == 0)  {modbus_status = 1;}
      if (modbus_use == 1 and error_link == 1)  {modbus_status = 2;}

      SD.cnt = dos_cnt;

      if (LE.Result > M2_High_SP and mix_2_speed == 1)
      {
        LE_Ok = 0;
      }
      if ((LE.Result > M2_Low_SP and LE.Result < M2_High_SP) or mix_2_speed == 0)
      {
        LE_Ok = 1;
      }

      LE_Control();
      Mix_Control();
      DS_Control();
      TH_Control();
      TS_Control();
      SD_Calibration();
      SD_Control();
    }
    //Приготовление - заполнение
      void LE_Control()
      {
        //Отключение установки
        if (CMD_Start == 0 or work_alarms[2] == 1) {LE_Step = 0; KC.Start_A = 0; T_KC_Delay.Start = 0; kc_open = 0;}
        //Шаг 0 - начало приготовления - контроль верхнего рабочего уровня при запуске
        if (CMD_Start == 1 and LE_Step == 0 and work_alarms[2] == 0)
        {
          if (LE.LSx[2] == 0) {KC.Start_A = 1; kc_open = 1;}
          LE_Step++;
        }
        //Шаг 1 - закрытие клапана при достижении верхнего рабочего уровня
        if (LE_Step == 1)
        {
          if (LE.LSx[2] == 1) {T_KC_Delay.Start = 1; kc_open = 0; LE_Step++;}
        }
        if (LE_Step == 2)
        {
          if (T_KC_Delay.Exit == 1) {T_KC_Delay.Start = 0; KC.Start_A = 0; LE_Step++;}
        }
        //Шаг 3 - открытие клапана при достижении нижнего рабочего уровня
        if (LE_Step == 3)
        {
          if (LE.LSx[1] == 1) {KC.Start_A = 1; kc_open = 1; LE_Step = 1;}
        }
      }

    //Приготовление - перемешивание
      void Mix_Control()
      {
        if (CMD_Start == 0)  {Mix_Step = 0; M1.Start_A = 0; M2.Start_A = 0; M3.Start_A = 0; T_M1.Start = 0; T_M2.Start = 0; T_M3.Start = 0;}
        if (CMD_Start == 1)
        {
          if (LE_Step == 1 or mix_123_use == 1) {Mix_Step = 1;}
          if (LE_Step != 1 and mix_123_use == 0 and Mix_Step == 0) {Mix_Step = 1;}
          if (Mix_Step == 1) 
          {
            T_M1.Start = 1;
            T_M2.Start = 0;
            T_M3.Start = 0; 
            if (T_M1.Exit == 1 and LE_Step == 2) {Mix_Step++;}
            if (mix_1_use == 1) {M1.Start_A = 1;} else {M1.Start_A = 0; Mix_Step++;}
            if (mix_2_use == 1 and (mix_12_use == 1 or mix_123_use == 1 or (mix_2_speed == 1 and LE_Ok == 1))) {M2.Start_A = 1;} else {M2.Start_A = 0;}
            if (mix_3_use == 1 and mix_123_use == 1) {M3.Start_A = 1;} else {M3.Start_A = 0;}
          }
          if (Mix_Step == 2) 
          {
            T_M1.Start = 0;
            T_M2.Start = 1;
            T_M3.Start = 0; 
            if (T_M2.Exit ==1) {Mix_Step++;}
            if (mix_1_use == 1 and (mix_12_use == 1 or mix_123_use == 1)) {M1.Start_A = 1;} else {M1.Start_A = 0;}
            if (mix_2_use == 1 and mix_2_speed == 0) {M2.Start_A = 1;} else {M2.Start_A = 0; Mix_Step++;}
            if (mix_3_use == 1 and mix_123_use == 1) {M3.Start_A = 1;} else {M3.Start_A = 0;}
          }
          if (Mix_Step == 3) 
          {
            T_M1.Start = 0;
            T_M2.Start = 0;
            T_M3.Start = 1; 
            if (T_M3.Exit ==1) {Mix_Step++;}
            if (mix_1_use == 1 and mix_123_use == 1) {M1.Start_A = 1;} else {M1.Start_A = 0;}
            if (mix_2_use == 1 and mix_123_use == 1) {M2.Start_A = 1;} else {M2.Start_A = 0;}
            if (mix_3_use == 1) {M3.Start_A = 1;} else {M3.Start_A = 0; Mix_Step++;}
          }
          if (Mix_Step == 4)
          {
            T_M1.Start = 0;
            T_M2.Start = 0;
            T_M3.Start = 0; 
            Mix_Step = 1;
          }
        }
      }

    //Приготовление - вибратор на буфере
      void DS_Control()
      {
        if (CMD_Start == 0 or DS_use == 0 or work_alarms[4] == 1) {DS_Step = 0; DS.Start_A = 0; T_DS_Wait_OFF.Start = 0; T_DS_Wait_ON.Start = 0; T_DS_Work_OFF.Start = 0; T_DS_Work_ON.Start = 0; DS_S1 = 0; DS_S2 = 0;}
        if (CMD_Start == 1 and DS_use == 1 and work_alarms[4] == 0)
        {
          if (KC.Start_A == 1) {DS_Step = 1;}
          if (KC.Start_A == 0) {DS_Step = 2;}
          if (DS_Step == 1)
          {
            if (DS_S1 == 0) {DS.Start_A = 1; T_DS_Work_ON.Start = 1; T_DS_Work_OFF.Start = 0; if (T_DS_Work_ON.Exit == 1)   { DS_S1 = 1;} }
            if (DS_S1 == 1) {DS.Start_A = 0; T_DS_Work_ON.Start = 0; T_DS_Work_OFF.Start = 1; if (T_DS_Work_OFF.Exit == 1)  { DS_S1 = 0;} }
          }
          if (DS_Step == 2)
          {
            if (DS_S2 == 0) {DS.Start_A = 1; T_DS_Wait_ON.Start = 1; T_DS_Wait_OFF.Start = 0; if (T_DS_Wait_ON.Exit == 1)   { DS_S2 = 1;} }
            if (DS_S2 == 1) {DS.Start_A = 0; T_DS_Wait_ON.Start = 0; T_DS_Wait_OFF.Start = 1; if (T_DS_Wait_OFF.Exit == 1)  { DS_S2 = 0;} }
          }
        }
      }

    //Приготовление - поддержание уровня буфера
      void TH_Control()
      {
        if (CMD_Start == 0 or TH_use == 0 or work_alarms[5] == 1) {TH_Step = 0; TH.Start_A = 0; T_TH_ON.Start = 0; T_LLS_delay.Start = 0; T_HLS_delay.Start = 0; TS_REQ = 0;}
        if (CMD_Start == 1 and TH_use == 1 and work_alarms[5] == 0)
        {
          //Определение пустого бункера
          if (TH_Step == 0 and LS.Status == 0)
          {
            T_LLS_delay.Start = 1;
            if (T_LLS_delay.Exit == 1)  {TH_Step++; if (TS_use == 1) {TS_Step = 1;}}
          }
          //Запуск шнека
          if (TH_Step == 1)
          {
            TH.Start_A = 1;
            if (TS_REQ == 0 and TS_use == 1) {TS_Step = 1; TS_REQ = 1;}
            if ((HLS.Use == 0 and LS.Status == 1) or (HLS.Use == 1 and LS.Status == 2)) {T_HLS_delay.Start = 1;}
            if (T_HLS_delay.Exit == 1) {TH_Step++;}
          }
          //Отключение шнека
          if (TH_Step == 2)
          {
            TH.Start_A = 0;
            TS_REQ = 0;
            T_LLS_delay.Start = 0;
            T_HLS_delay.Start = 0;
            TH_Step = 0;
          }
        }
      }  

    //Приготовление - вибратор на бункере
      void TS_Control()
      {
        if (CMD_Start == 0 or TS_use == 0) {TS_Step = 0; TS.Start_A = 0; T_TS_ON.Start = 0; T_TS_OFF.Start = 0;}
        if (CMD_Start == 1 and TS_use == 1)
        {
          //Время между пусками
          if (TS_Step == 0)
          {
            TS.Start_A = 0;
            T_TS_OFF.Start = 1;
            if (T_TS_OFF.Exit == 1)  {TS_Step++;}
          }
          //Время работы
          if (TS_Step == 1)
          {
            TS.Start_A = 1;
            T_TS_ON.Start = 1;
            if (T_TS_ON.Exit == 1)  {TS_Step++;}
          }
          //Возврат на нулевой шаг
          if (TS_Step == 2)
          {
            T_TS_OFF.Start = 0; T_TS_ON.Start = 0; TS.Start_A = 0; TS_Step = 0;
          }  
        }
      }

    //Приготовление - калибровка дозатора
      void SD_Calibration()
      {
        if (CMD_Start == 1) {SD.Calibration = 0;}
        if (CMD_Start == 0 and SD.Calibration == 1)
        {
          SD.REQ_Distance = dos_c_distance;
          SD.REQ_Speed = dos_c_speed;
          SD.status_vis = SD.Status;
        }
      }

    //Приготовление - дозация
      void SD_Control()
      {
        SD.calc();

        if ((CMD_Start == 0 and SD.Test == 0 and SD.Calibration == 0) or kc_open == 0 or (mix_2_speed == 1 and (LE.Result < M2_Low_SP or LE.Result > M2_High_SP))) {dos_cnt = 0; dos_cnt_add = 0; dos_cnt_sub = 0; SD.Start = 0;}

        //Расчёт массы на 1 оборот
        dos_c_one = dos_c_mass / dos_c_distance;
        //Расчёт дистанции на 1 импульс расходомера
        dos_c_impulse = ((SP_C * 10 * RM.Weight) / dos_c_one) * (100 / dos_c_source);

        if (CMD_Start == 1 and kc_open == 1 and (mix_2_speed == 0 or (mix_2_speed == 1 and LE_Ok == 1)))
        {
          //дозация по импульсам расходомера
            if (SD.Adapt == 0 or dos_cnt < SD.cnt_min)
            {

              //+1 доза
              if (Flow_RM.Status == 1 and dos_cnt_add == 0)
              {
                dos_cnt++;
                dos_cnt_add = 1;
              }
              if (Flow_RM.Status == 0)
              {
                dos_cnt_add = 0;
              }

              //-1 доза
              if (SD.Start == 0 and dos_cnt > 0 and dos_cnt_sub == 0)
              {
                //задание скорости и дистанции
                SD.REQ_Distance = dos_c_impulse;
                SD.REQ_Speed = dos_c_speed;
                SD.Start = 1;
                dos_cnt_sub = 1;
              }
              if (SD.end == 1 and dos_cnt_sub == 1)
              {
                dos_cnt_sub = 0;
                SD.Start = 0;
                if (SD.Adapt == 0){dos_cnt--;}
              }
            }
          //дозация по скорости протока
            if (SD.Adapt == 1 and dos_cnt >= SD.cnt_min)
            {
              if (RM.Flow > 0)  { SD.REQ_Speed = ((RM.Flow / 60) * (SP_C * 10)) / dos_c_one;  } else {SD.REQ_Speed = 0;}
              SD.Start = 1;
            }
        }
      }

//===========> КОНТРОЛЬ АВАРИЙ
  void Fault_Control()
  {
    Fault_Devices();
    Fault_Process();
    Fault_ModBus();

    if (alarm_devices == 1) {any_alarm = 1;}
    for (int i = 0; i <=7; i++)
      {
        if (work_alarms[i] == 1) {any_alarm = 1;}
      }
    if (Reset_All == 1)
    {
      alarm_devices = 0;
      any_alarm = 0;
      for (int i = 0; i <=7; i++)
      {
        work_alarms[i] = 0;
        T_Alm[i].Start = 0;
      }
      dos_cnt = 0;
    }
  }

  //Аварии устройств
    void Fault_Devices()
    {
      if (Reset_All == 0)
      {
        for (int i = 0; i <= 3; i++)
        {
          if (M1.Alarms[i] == 1)  {alarm_devices = 1;}
          if (M2.Alarms[i] == 1)  {alarm_devices = 1;}
          if (M3.Alarms[i] == 1)  {alarm_devices = 1;}
          if (DS.Alarms[i] == 1)  {alarm_devices = 1;}
          if (TH.Alarms[i] == 1)  {alarm_devices = 1;}
          if (TS.Alarms[i] == 1)  {alarm_devices = 1;}
        }
      }
    }

  //Аварии техпроцесса  //0 - сухой ход, 1 - перелив, 2 - нет протока, 3 - нет заполнения, 4 - буфер пуст, 5 - бункер пуст, 6 - дозация, 8 - разрешение, 7 - внешняя блокировка
      void Fault_Process()
      {
        if (Reset_All == 0)
        {
          if (LE.LSx[0] > 0)  { T_Alm[0].Start = 1;} else {T_Alm[0].Start = 0;}
          if (LE.LSx[3] > 0)  { T_Alm[1].Start = 1;} else {T_Alm[1].Start = 0;}
          if (KC.Start_M == 1 and Flow_RM.Status == 0) { T_Alm[2].Start = 1;} else {T_Alm[2].Start = 0;}
          if (KC.Start_M == 1 and LE.update() == 0) { T_Alm[3].Start = 1;} else {T_Alm[3].Start = 0;}
          if (LLS.Use == 1 and LLS.Status == 0)  { T_Alm[4].Start = 1;} else {T_Alm[4].Start = 0;}
          if (ALS.Use == 1 and ALS.Status == 0)  { T_Alm[5].Start = 1;} else {T_Alm[5].Start = 0;}
          if (TH.Status == 1) {T_TH_Limit.Start = 1;} else {T_TH_Limit.Start = 0;}
          
          if (T_Alm[0].Exit == 1) {work_alarms[0] = 1;}
          if (T_Alm[1].Exit == 1) {work_alarms[1] = 1;}
          if (T_Alm[2].Exit == 1) {work_alarms[2] = 1;}
          if (T_Alm[3].Exit == 1) {work_alarms[3] = 1;}
          if (T_Alm[4].Exit == 1) {work_alarms[4] = 1;}
          if (T_Alm[5].Exit == 1 or T_TH_Limit.Exit == 1) {work_alarms[5] = 1;}

          if (CMD_Start == 1 and dos_cnt > dos_cnt_max) {work_alarms[6] = 1;} else {work_alarms[6] = 0;}
          if (Block.Status == 0)  {work_alarms[7] = 1;}

          if (Start_REQ.Status == 1) {work_alarms[8] = 1;} else  {work_alarms[8] = 0;}
        }
      }

  //Аварии коммуникации
      void Fault_ModBus()
      {
        if (modbus_use == 1)
        {
          if (WD_in == WD_in_last)  {T_ModBus.Start = 1;}
          if (WD_in != WD_in_last)  {T_ModBus.Start = 0; WD_in_last = WD_in;}
          if (T_ModBus.Exit == 1)   {error_link = 1;} else {error_link = 0;}
        }
        else
        {
          error_link = 0;
          T_ModBus.Start = 0;
          WD_in = 0;
          WD_in_last = 0;
        }
      }

//============> MODBUS RTU
  //обработка связи по сети
    void communication()
    {
      //WatchDog
        if (WatchDog < 1000 and pulse_1s)  {WatchDog++;}
        if (WatchDog >= 1000 and pulse_1s) {WatchDog = 0;}

      //Отправка данных о состоянии установки
        MB_Data[0] = 100 + mixers*10 + sections;   //код устаноовки - [тип 1 - приготовление, 2 - дозарование], [число линий основных/число мешалок], [число линий резервных/транспортный шнек]
        MB_Data[1] = REQ_Start*100 + Start_REQ.Status*10 + CMD_Start;
        MB_Data[2] = M1.Status;
        MB_Data[3] = M2.Status;
        MB_Data[4] = M3.Status;
        MB_Data[5] = DS.Status;
        MB_Data[6] = KC.Status;
        MB_Data[7] = TH.Status;
        MB_Data[8] = TS.Status;
        MB_Data[9] = SD.Status;
        MB_Data[11] = int(SD.REQ_Speed*10);
        MB_Data[12] = int(RM.Flow);
        MB_Data[13] = int(LE.Result*10);
        MB_Data[14] = int(SP_C*10);
        MB_Data[15] = LS.Status;
        MB_Data[16] = work_alarms[7]*10000 + any_alarm*1000 + work_alarms[4]*100 + work_alarms[0]*10 + work_alarms[2];
        MB_Data[17] = PolyMix_ID;
        MB_Data[18] = WatchDog;

      //Управение установкой по сети
        if (modbus_use == 1)
        {
          REQ_Start = MB_Data[20];
          SP_C = float(MB_Data[21]/10);
          Reset_All = MB_Data[22];
          WD_in = MB_Data[23];
        }

      state = slave.poll(MB_Data, 50); 
    }

uint32_t test_get;
uint32_t test_val;
int x_test = 0;
//>>>>>>>>>>>>>>главный цикл
  void loop()
  { 
    drv_pulse();
    nexLoop(nex_listen_list);
    signals();
    PolyMix();
    devices();
    Fault_Control();
    HMI();
    communication();
    drv_end();
    wdt_reset();
  }
