package edu.fudan.fybody.action;

import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.struts2.json.annotations.JSON;
import org.hibernate.Criteria;
import org.hibernate.Query;
import org.hibernate.Session;
import org.hibernate.Transaction;
import org.hibernate.criterion.Restrictions;

import com.opensymphony.xwork2.ActionSupport;

import edu.fudan.fybody.domain.Record;
import edu.fudan.fybody.domain.User;
import edu.fudan.fybody.util.DateUtil;
import edu.fudan.fybody.util.HibernateSessionFactory;

public class GetRecordsAction extends ActionSupport {

	/**
	 * 
	 */
	private static final long serialVersionUID = 8901684011611289387L;

	private String uname;
	private int year;
	private int month;
	private Map<String, String> map;
	
	@JSON(serialize = false)
	public String getUname() {
		return uname;
	}
	public void setUname(String uname) {
		this.uname = uname;
	}
	
	@JSON(serialize = false)
	public int getYear() {
		return year;
	}
	public void setYear(int year) {
		this.year = year;
	}
	
	@JSON(serialize = false)
	public int getMonth() {
		return month;
	}
	public void setMonth(int month) {
		this.month = month;
	}
	
	public Map<String, String> getResult() {
		return map;
	}
	
	public String execute() throws Exception {
		
		Session session = HibernateSessionFactory.getSession();
		map = new HashMap<String, String>();
		Transaction tx = null;
		
		User user = new User();
		
		try {
			Criteria ucrit = session.createCriteria(User.class);
			ucrit.add(Restrictions.eq("account", uname));
			user = (User) ucrit.uniqueResult();
			
			int uid = user.getId();			
			String hql ="from Record where created_at between ? and ? and user_id = :user_id"; 
			Query query = session.createQuery(hql);
			query.setParameter(0, DateUtil.getMonthStartDate(year, month));
			query.setParameter(1, DateUtil.getMonthEndDate(year, month));
			query.setParameter("user_id", uid);
			
			List<Record> records = query.list();
			
			int maxDays = DateUtil.getMaxDayOfMonth(year, month);
			int days[] = new int[maxDays];
			
			for(int i = 0; i < days.length; i++) {
				days[i] = 0;
			}
			
			for(Record record : records) {
				Date date = record.getCreated_at();
				days[DateUtil.getDayByDate(date) - 1] = 1;
			}				
			
			StringBuilder response = new StringBuilder();
			for(int i = 0; i < days.length; i++) {
				response.append(days[i]);
				response.append(",");
			}
			response.deleteCharAt(response.length() - 1);
			
			map.put("success", "true");
			map.put("response", response.toString());
		}
		catch (Exception e) {
			map.put("success", "false");
			e.printStackTrace();
		}
		finally {
			session.close();
		}
		
		return SUCCESS;
	}
}
