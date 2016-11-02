void mode_adc(void)
{
	char str[32];
	unsigned char ctr=0;
	unsigned char topline = 20;
	unsigned long wu=0;
	unsigned char hour,min,sec=0,lastsec=0;
	unsigned short temp;
	unsigned long conv=0;
	
	lcd_clear565(0);
	
	fprintf_P(file_pri,PSTR("Mode ADC\n"));
	
	ADCSetPrescaler(ADCCONV_PRESCALER_64);
	
	//set_sleep_mode(SLEEP_MODE_IDLE); 
	
	WAITPERIOD p=0;
	unsigned short d=0;
	unsigned long int lt=0;
	unsigned short v[8];
	unsigned short r;
	unsigned char i=0;
	unsigned long t1,t2,t;
	
	
	
	
	
	/*unsigned long tms,tus;
	while(1)
	{
		tms=timer_ms_get();
		tus=timer_us_get();
		fprintf_P(file_pri,PSTR("%lu %lu\n"),tms,tus);
		//printf("%lu\n",tus);
	}
	*/
	
	
	_delay_ms(1000);
	unsigned long int tt[100];
	unsigned long int tt2[100];
	for(int i=0;i<100;i++)
		tt[i] =	timer_ms_get();
	fprintf(file_pri,"Successive timer_ms_get: ");
	for(int i=0;i<100;i++)
		fprintf(file_pri,"%ld ",tt[i]);
	fprintf(file_pri,"\n");
	_delay_ms(1000);
	for(int i=0;i<100;i++)
		tt[i] =	timer_us_get();
	fprintf(file_pri,"Successive timer_us_get: ");
	for(int i=0;i<100;i++)
		fprintf(file_pri,"%ld ",tt[i]);
	fprintf(file_pri,"\n");
	fprintf(file_pri,"Successive timer_us_get delta: ");
	for(int i=1;i<100;i++)
		fprintf(file_pri,"%ld ",tt[i]-tt[i-1]);
	fprintf(file_pri,"\n");
	
	fprintf(file_pri,"Benchmark timer_ms_get: ");
	t1 = timer_ms_get();
	for(unsigned i=0;i<1000;i++)
		t=timer_ms_get();
	t2 = timer_ms_get();
	fprintf(file_pri,"Time for 1000: %ld\n",t2-t1);

	fprintf(file_pri,"Benchmark timer_us_get: ");
	t1 = timer_ms_get();
	for(unsigned i=0;i<1000;i++)
		t=timer_us_get();
	t2 = timer_ms_get();
	fprintf(file_pri,"Time for 1000: %ld\n",t2-t1);	
	
	fprintf(file_pri,"OCR1A: %d\n",OCR1A);	
	
	
	_delay_ms(1000);
	p=0;
	d=0;
	for(int i=0;i<100;i++)
	{
		tt[i] =timer_waitperiod_us(100,&p);
		tt2[i] = p;
		//tt[i] =timer_waitperiod_ms(1,&p);
		/*_delay_us(d);
		d+=100;
		if(d>3500)
			d=0;*/
		
	}
	
	for(int i=1;i<100;i++)
	{
		fprintf(file_pri,"%lu %lu. %lu %lu\n",tt[i-1],tt2[i-1],tt[i]-tt[i-1],tt[i]-tt[0]);
	}
	
	while(1);
	
	
	while(1)
	{
		/*timer_waitperiod_ms(100,&p);
		
		unsigned long int t = timer_ms_get();
		fprintf(file_pri,"t: %ld, d: %d (%ld)\n",t,d,t-lt);
		unsigned long int t2 = timer_ms_get();
		fprintf(file_pri,"printf time: %ld\n",t2-t);
		lt=t;
		_delay_ms(d);
		
		t2 = timer_ms_get();
		fprintf(file_pri,"time aft delay: %ld\n",t2);
		
		d+=10;
		
		if(d>250)
			d=0;
		*/
		
		fprintf_P(file_pri,PSTR("Some ADC stuff: %d. period: %d. mask: %02X. pri: %p dbg: %p\n"),ctr,mode_adc_period,mode_adc_mask,file_pri,file_dbg);
		
		// Period
		timer_waitperiod_ms(mode_adc_period,&p);
		
		while(CommandProcess(CommandParsersADC,CommandParsersADCNum));
		
		if(CommandShouldQuit())
			return 1;
			
		
		// Convert adc 
		ADCChainedRead(mode_adc_mask,v);
		// Print it
		for(unsigned i=0;i<__builtin_popcount(mode_adc_mask);i++)
			fprintf_P(file_pri,PSTR("%d "),v[i]);
		fprintf_P(file_pri,PSTR("\n"));
		
		/*unsigned char t = mo