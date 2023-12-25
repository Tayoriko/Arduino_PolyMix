class DRV_SD
{
	public:
		void config(int pin_Imp, int pin_Vector, int pic_Lock);
		void exe();
		void calc();
		int update();
		int Start = 0;
		int Test = 0;
		int Cancel = 0;
		int Calibration = 0;
		int Adapt = 0;
		int Vector = 0;
		float SP_imp = 400;	//количество импульсов на 1 оборот
		float SP_ms = 256;	//скорость подачи импульсов для 1 оборота в минуту
		float SP_Reduction = 10.0;	//передаточное число редуктора
		float REQ_Distance = 0;
		float REQ_Speed = 0;
		int test_vis = 0;
		int adapt_vis = 0;
		int vector_vis = 0;
		int status_vis = 0;
		int Status = 0;
		int cnt_min = 3;
		int cnt = 0;
		int Step = 0;
		float speed = 0;
		float Delta_SP = 0;
		float Delta_SP_Now = 0.25;
		float Last_Speed = 0;
		unsigned long _micros = 0;
		unsigned long last_micros = 0;
		unsigned long delta = 0;
		unsigned long distance = 0;
		unsigned long imp = 0;
		unsigned long turn = 0;
		int CMD = 0;
		bool end = 0;
	private:
		float SP_Max = 22.5;
		int _pin_Imp = 0;
		int _pin_Vector = 0;
		int _pin_Lock = 0;
		
};

void DRV_SD::config(int pin_Imp, int pin_Vector, int pic_Lock)
{
	if (pin_Imp > 0) 		{pinMode(pin_Imp, OUTPUT);}
	if (pin_Vector > 0) 	{pinMode(pin_Vector, OUTPUT);}
	if (pic_Lock > 0) 		{pinMode(pic_Lock, OUTPUT);}
	_pin_Imp = pin_Imp;
	_pin_Vector = pin_Vector;
	_pin_Lock = pic_Lock;
}

void DRV_SD::calc()
{
	if (REQ_Speed > SP_Max) {REQ_Speed = SP_Max;}
	//Отмена всех операций
	if (Cancel == 1)
	{
		Test = 0;
		Calibration = 0;
		Cancel = 0;
		Step = 0;
		CMD = 0;
		Start = 0;
	}

	//Тестовый оборот
	  	if (Test == 1) {REQ_Distance = 1.0; REQ_Speed = 1.0; Start = 1;}
	  	if (Calibration == 1) {Start = 1;}


	  	//Управление вращением в импульсном режиме и при отладке
	  	if (Start == 1 and (Adapt == 0 or cnt < cnt_min))
	  	{
	  		//рассчёт параметров
		  		speed = REQ_Speed / 60;								//количество импульсов в секунду для поддержания скорости
		  		distance = REQ_Distance * SP_imp * SP_Reduction;	//количество импульсов на один оборот вращения шнека
		  		imp = SP_ms / speed;								//фактическое время между импульсами
		  	}
		  	//Управление вращением в постоянном режиме
	  	if (Start == 1 and Adapt == 1 and cnt >= cnt_min)
	  	{
	  		speed = REQ_Speed / 60;
	  		imp = SP_ms / speed;
	  	}

	  	if (_pin_Lock > 0)		{ if (Start == 1) {digitalWrite(_pin_Lock, LOW);} else {digitalWrite(_pin_Lock, HIGH);} }
  		if (_pin_Vector > 0)	{ digitalWrite(_pin_Vector, Vector);}

  		if (Start == 1) {Status = 1;} else {Status = 0; end = 0;}

  		Delta_SP = fabs(REQ_Speed - Last_Speed);

}

void DRV_SD::exe()
{
	_micros = micros();

	//контроль текущего времени - защите от переполнения
	  	if (_micros < last_micros)	{ last_micros = _micros; }
	  	delta = _micros - last_micros;

  	//Управление вращением в импульсном режиме и при отладке
	  	if (Start == 1 and (Adapt == 0 or cnt < cnt_min))
	  	{
	  		//подача импульса
			    if (Step == 0)
				    {
				      turn = 0;
				      Step = 1;
				    }
			    if (Step == 1)
				    {
				      if ((delta > imp) and (turn < distance))	//подсчёт количества импульсов для контроля пройденного расстояния
					      {
					        CMD = 1;
					        turn = turn + 1;
					        last_micros = _micros;
					      }
				      else
					      {
					        CMD = 0;
						  }
				      if (turn >= distance)
					      {
					        CMD = 0;
					        Step = 2;
					      }
				    }
				if (Step == 2)
				    {
				      CMD = 0;
				      Step = 0;
				      //Start = 0;
				      end = 1;
				      Test = 0;
				      Calibration = 0;
				    }
	  	}

	//Управление вращением в постоянном режиме
	  	if (Start == 1 and Adapt == 1 and cnt >= cnt_min)
	  	{
			if (delta > imp)	//подсчёт количества импульсов для контроля пройденного расстояния
			{
				CMD = 1;
				last_micros = _micros;
			}
			if (delta <= imp)
			{
				CMD = 0;
			}
	  	}

  	if (CMD == 1)
  	{
  		PORTH = B00001000;
  	}
  	if (CMD == 0)
  	{
  		PORTH = B00000000;
  	}
  	

  	
}

	//Обновление данных для панели
		int DRV_SD::update()
		{
			if (Delta_SP > Delta_SP_Now)
			{
				Last_Speed = REQ_Speed;
				return 1;
			}
			else
			{
				return 0;
			}
		}