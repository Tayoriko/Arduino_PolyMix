//Двигатель с прямым пуском
	//Состав драйвера
		class DRV_MD
		{
			public:
				void config(int pin_QF, int pin_KM, int pin_NoErr, int pin_CMD);
				void exe(int clock, int Reset);
				int Hand = 0;
				int Start_M = 0;
				int Start_A = 0;
				int Alm;
				int hand_vis = 0;
				int start_vis = 0;
				int alm_vis;
				int Status = 0;
				int status_vis = 0;
				int Alarms[5];			//0 - QF, 1 - Неиспр, 2 - запуск, 3 - останов
				int alarms_vis[5];

			private:
				int QF;	//ОС "Защитный автомат включен"/"Устройство готово"
				int KM;	//ОС "В работе"
				int NoErr;	//ОС "Авария"
				int CMD = 0;	//Команда "Пуск/Стоп"
				int CMD_set = 0;
				int SP_ON = 2;	//время ожидания появления ОС "В работе" при запуске
				int SP_OFF = 2;	//время ожидания исчезновения ОС "В работе" при останове
				int Now_ON = 0;
				int Now_OFF = 0;
				int _pin_QF;
				int _pin_KM;
				int _pin_NoErr;
				int _pin_CMD;

		};

	//конфигурация привода
		void DRV_MD::config(int pin_QF, int pin_KM, int pin_NoErr, int pin_CMD)
		{
			_pin_QF = pin_QF;
			_pin_KM = pin_KM;
			_pin_NoErr = pin_NoErr;
			_pin_CMD = pin_CMD;
		}


	//формирование статуса
		void DRV_MD::exe(int clock, int Reset)
		{
			if (Reset == 0)
			{
				if (_pin_QF > 0)	{if (digitalRead(_pin_QF) == 0) 	{QF = 1;} 		else {QF = 0;}} 	else {QF = 1;}
				if (_pin_NoErr > 0) {if (digitalRead(_pin_NoErr) == 0) 	{NoErr = 1;}	else {NoErr = 0;}} 	else {NoErr = 1;}
				if (_pin_KM > 0) 	{if (digitalRead(_pin_KM) == 0) 	{KM = 1;} 		else {KM = 0;}} 	else {KM = CMD;}
				//контроль аварий
				if (_pin_QF > 0 and QF == 0) 			{Status = 5; Alarms[0] = 1;}
				if (_pin_NoErr > 0 and NoErr == 0)		{Status = 4; Alarms[1] = 1;}
			}

			if (Alarms[0] > 0 or Alarms[1] > 0 or Alarms[2] > 0 or Alarms[3] > 0 or Alarms[4] > 0)	{Alm = 1;}
			if (Reset == 1)	{Alarms[0] = 0; Alarms[1] = 0; Alarms[2] = 0; Alarms[3] = 0; Alarms[4] = 0; Alm = 0; Now_ON = 0; Now_OFF = 0;}

			//управление в авторежиме
			if (Hand == 0)
			{
				if (Start_A == 1 and Alm == 0) 	{CMD = 1;}
				if (Start_A == 0 or Alm == 1)  	{CMD = 0;}
				Start_M = Start_A;
			}

			//управление в ручном режиме
			if (Hand == 1)
			{
				if (Start_M == 1 and Alm == 0) 	{CMD = 1;}
				if (Start_M == 0 or Alm == 1) 	{CMD = 0;}
			}

			//управление выходом
			if (Alm == 0 and CMD == 1 and _pin_CMD > 0)
			{
				if (CMD != CMD_set)
				{
					digitalWrite(_pin_CMD, HIGH);
					CMD_set = CMD;
				}				
			}
			else
			{
				if (CMD != CMD_set)
				{
					digitalWrite(_pin_CMD, LOW);
					CMD_set = CMD;
				}
			}

			if (CMD == 0 and Alm == 0)
			{	//контроль останова
				Now_ON = 0;
				if (Now_OFF < SP_OFF and clock == 1){Now_OFF++; Status = 2;}
				if ((Now_OFF >= SP_OFF and KM == 0) or _pin_KM == 0) {Status = 0;}
				if (Now_OFF >= SP_OFF and KM == 1){Status = 6; Alarms[2] = 1;}
			}
			if (CMD == 1 and Alm == 0)
			{	//контроль запуска
				Now_OFF = 0;
				if (Now_ON < SP_ON and clock == 1){Now_ON++; Status = 3;}
				if ((Now_ON >= SP_ON and KM == 1) or _pin_KM == 0) {Status = 1;}
				if (Now_ON >= SP_ON and KM == 0){Status = 7; Alarms[3] = 1;}
			}
			
		}

//Дискретный вход
	//Состав драйвера
		class DRV_DI
		{
			public:
				void config(int pin_DI);
				void exe(int clock, int Filter_SP);
				int Use = 0;
				int Inv = 0;
				int Status = 0;
				int status_vis = 0;
				int use_vis = 0;
				int inv_vis = 0;
				int Data = 0;	
			private:	
				int Now_ON = 0;
				int Now_OFF = 0;
				int _pin_DI;
		};

	//Конфигурация входа
		void DRV_DI::config(int pin_DI)
		{
			_pin_DI = pin_DI;
		}

	//Чтение входа
		void DRV_DI::exe(int clock, int Filter_SP)
		{
			//Чтение входа
			if (_pin_DI > 0) {Data = digitalRead(_pin_DI);} else {Data = 0;}

			if (Data == 1 and Now_ON < Filter_SP and clock == 1){Now_ON++;}
			if (Data == 1 and Now_ON >= Filter_SP)	{Now_OFF = 0; if (Inv == 0){Status = 0;}else{Status = 1;}}

			if (Data == 0 and Now_OFF < Filter_SP and clock == 1){Now_OFF++;}
			if (Data == 0 and Now_OFF >= Filter_SP)	{Now_ON = 0; if (Inv == 0){Status = 1;}else{Status = 0;}}
		}

//Дискретный выход
	//Состав драйвера
		class DRV_DO
		{
			public:
				void config(int pin_DO);
				void exe(int Start_A);
				int Use = 0;
				int Inv = 0;
				int Status = 0;
				int status_vis = 0;
				int Hand = 0;
				int Start_M = 0;
				int hand_vis = 0;
				int start_vis = 0;
				int use_vis = 0;
				int inv_vis = 0;
				int Data = 0;	
			private:
				//int Data = 0;	
				int Now_ON = 0;
				int Now_OFF = 0;
				int _pin_DO;
		};

	//Конфигурация входа
		void DRV_DO::config(int pin_DO)
		{
			_pin_DO = pin_DO;
		}

	//Чтение входа
		void DRV_DO::exe(int Start_A)
		{
			//управление в авторежиме
			if (Hand == 0)
			{
				Start_M = Start_A;
			}

			if (Start_M == 1)	{digitalWrite(_pin_DO, HIGH);}else{digitalWrite(_pin_DO, LOW);}
		}

//Аналоговый вход
	//Состав драйвера
		class DRV_AI
		{
			public:
				void config(int pin_AI, float _delta_sp);
				void exe(int F_Mode, int Reset, int clock);
				int update();
				float Result = 0;
				float min_in = 204;
				float max_in = 850;
				float min_out = 0;
				float max_out = 100;
				int LSx[6]; 	//0 - аварийно низкий, 1 - нижний рабочий, 2 - верхний рабочий, 3 - аварийно высокий
				int LSx_vis[6];
				float HHS = 95;
				float HS = 85;
				float LS = 30;
				float LLS = 15;
			private:
				#define SIZE_AI 3
				int _pin_AI;
				int Signal;
				float Delta = 0;
				float Delta_SP = 1;
				float Value;
				float Last_Result;
				float Summ;
				float Median;
				float DATA_AI[SIZE_AI];
		};

	//Чтение входа
		void DRV_AI::config(int pin_AI, float _delta_sp)
		{
			if (pin_AI > 0) {_pin_AI = pin_AI;}
			Delta_SP = _delta_sp;
		}

	//Обработка сигнала
		void DRV_AI::exe(int F_Mode, int Reset, int clock)
		{
			//Чтение сигнала
			Signal = analogRead(_pin_AI);
			
			Value = (Signal - min_in) / ((max_in - min_in) / (max_out - min_out)) + min_out;

			if (Value < min_out)	{Value = min_out;}
			if (Value > max_out)	{Value = max_out;}
			//Режим без фильтрации
			if (F_Mode <= 0 or F_Mode > 2)
			{
				Result = Value;
			}

			if (F_Mode == 1)
			{
				DATA_AI[0] = Value;
				Summ = 0.0;
				for(int i=SIZE_AI; i>0; i--)
			  	{
			    	DATA_AI[i] = DATA_AI[i - 1];
			    	Summ += DATA_AI[i];    
			  	}
			  	Median = Summ / SIZE_AI;	
			  	Result = Median;
			}

			if (F_Mode == 2 and clock == 1)
			{
				DATA_AI[0] = Value;
				Summ = 0.0;
				for(int i=SIZE_AI; i>0; i--)
			  	{
			    	DATA_AI[i] = DATA_AI[i - 1];
			    	Summ += DATA_AI[i];    
			  	}
			  	Median = Summ / SIZE_AI;	
			  	Result = Median;				
			}

			Delta = fabs(Result - Last_Result);


			if (LSx[5] == 1) {Result = max_out - Result;}
			
			if (Result < LLS)	{LSx[0] = 1;}
			if (Result < LS)	{LSx[1] = 1;}	else	{LSx[1] = 0;}
			if (Result > HS)	{LSx[2] = 1;}	else	{LSx[2] = 0;}
			if (Result > HHS)	{LSx[3] = 1;}
			if (Signal < 100) 	{LSx[4] = 1;}
			if (Reset == 1) 	{LSx[0] = 0; LSx[3] = 0; LSx[4] = 0;}

		}

	//Обновление данных для панели
		int DRV_AI::update()
		{
			if (Delta > Delta_SP)
			{
				Last_Result = Result;
				return 1;
			}
			else
			{
				return 0;
			}
		}

//Секундный таймер
	//Состав таймера
		class TON_s
		{
			public:
				void exe(int clock);
				int Start = 0;
				int Exit = 0;
				float tSP = 5;
			private:
				float NOW = 0;
		};

	//Выполнение таймера
		void TON_s::exe(int clock)
		{
			if (Start == 0)	{NOW =0; Exit = 0;}
			if (Start == 1)
			{
				if (NOW < tSP)
					{
						if (clock == 1)	{NOW = NOW + 0.1;}
						Exit = 0;
					}
				if (NOW >= tSP) {Exit = 1;}
			}
		}

//Минутный таймер
	//Состав таймера
		class TON_m
		{
			public:
				void exe(int clock);
				int Start = 0;
				int Exit = 0;
				float tSP = 1;
			private:
				float NOW = 0;
				float SP_m = 0;
		};

	//Выполнение таймера
		void TON_m::exe(int clock)
		{
			SP_m = tSP * 60.0;
			if (Start == 0)	{NOW =0; Exit = 0;}
			if (Start == 1)
			{
				if (NOW < SP_m)
					{
						if (clock == 1)	{NOW = NOW + 1;}
						Exit = 0;
					}
				if (NOW >= SP_m) {Exit = 1;}
			}
		}

//Расходомер
	//Состав расходомера
		class DRV_Flow
		{
			public:
				void exe(int KC_Status, int impulse, int clock);
				int update();
				float Flow = 0;
				float cnt = 0;
				float Weight = 1;
			private:
				#define SIZE_FL 20
				#define NoFlow_SP 10
				int SIZE = SIZE_FL - 1;
				float DATA_Flow[SIZE_FL];
				unsigned long DATA_Last = 0;
				unsigned long DATA_Now = 0;
				unsigned long Delta = 0;
				float DATA_Summ = 0;
				int _pin_DI_FL = 0;
				int data_clean = 0;
				int data_read = 0;
				int NoFlow = 0;
				float Flow_Last = 0;
				float Flow_Delta = 0;
				float Flow_Delta_SP = 25;
		};

	//Обработка сигнала
		void DRV_Flow::exe(int KC_Status, int impulse, int clock)
		{
			//Обнуление данных
			if (KC_Status == 0)
			{
				Flow = 0;
				if (data_clean == 0)
				{
					for (int i=SIZE_FL; i > 0; i--)
			        {
			            DATA_Flow[i] = 0;
			        }
			        data_clean = 1;
			        data_read = 0;
			        cnt = 0;
				}
				//DATA_Now = micros();
				//DATA_Last = DATA_Now;
			}

			//Проведение замеров
			if (KC_Status == 1)
			{
				data_clean = 0;

				//действия при поступлении импульса
				if (impulse == 1 and data_read == 0)
				{	
					//время между импульсами
					DATA_Last = DATA_Now;
					DATA_Now = micros();
					Delta = DATA_Now - DATA_Last;
					if (DATA_Last > DATA_Now)
					{
						Delta = 4294967295 - DATA_Last + DATA_Now;
					}

					//время в мс
					DATA_Flow[0] = float(Delta) / 1000;
					DATA_Summ = 0;

					if (DATA_Flow[0] > 100)
					{
						//смещение массива
						for (int i=0; i<SIZE_FL-1; i++)//i=SIZE_FL; i > 0; i--)
				        {
				            DATA_Flow[i+1] = DATA_Flow[i];
				            //DATA_Flow[i] = DATA_Flow[i-1];
				            if (i < cnt) {DATA_Summ = DATA_Summ + DATA_Flow[i];}
				        }
				        if (cnt < SIZE) {cnt++;}
				    }
			        
			        NoFlow = 0;
			        data_read = 1;
				}

				//действия после импульса
				if (impulse == 0 and data_read == 1)
				{
					data_read = 0;
				}

				//рассчёт скорости потока
				Flow = 3600 * (1.0 / ((DATA_Summ / cnt) / 1000.0));
				if (Flow < 0) {Flow = 0;}

				if (impulse == 0 and data_read == 0 and clock == 1)
				{
					if (NoFlow < NoFlow_SP)	{	NoFlow++;	}
					if (NoFlow >= NoFlow_SP)
					{
						Flow = 0;	
						for (int i=SIZE_FL; i > 0; i--)
				        {
				            DATA_Flow[i] = 0;
				        }
				        cnt = 0;
					}
				}
			}

			Flow_Delta = fabs(Flow - Flow_Last);
		}

	//Обновление данных для панели
		int DRV_Flow::update()
		{
			if (Flow_Delta > Flow_Delta_SP)
			{
				Flow_Last = Flow;
				return 1;
			}
			else
			{
				return 0;
			}
		}

