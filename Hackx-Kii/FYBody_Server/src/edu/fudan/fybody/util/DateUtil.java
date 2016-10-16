package edu.fudan.fybody.util;

import java.util.Calendar;
import java.util.Date;

public class DateUtil {

	public static Date getMonthStartDate(int year, int month) {
		Calendar cal = Calendar.getInstance();
		cal.set(year, month - 1, 1, 0, 0, 0);
		return cal.getTime();
	}
	
	public static Date getMonthEndDate(int year, int month) {
		Calendar cal = Calendar.getInstance();
		cal.set(year, month, 1, 0, 0, 0);
		return cal.getTime();
	}
	
	public static int getDayByDate(Date date) {
		Calendar cal = Calendar.getInstance();
		cal.setTime(date);	
		return cal.get(Calendar.DATE);
	}
	
	public static int getMaxDayOfMonth(int year, int month) {
		Calendar cal = Calendar.getInstance();
		cal.set(Calendar.YEAR, year);
		cal.set(Calendar.MONTH, month - 1);
		
		return cal.getActualMaximum(Calendar.DAY_OF_MONTH);
	}
}
