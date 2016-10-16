package edu.fudan.fybody.action;

import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.struts2.json.annotations.JSON;
import org.hibernate.Criteria;
import org.hibernate.Session;
import org.hibernate.Transaction;
import org.hibernate.criterion.Restrictions;

import com.opensymphony.xwork2.ActionSupport;

import edu.fudan.fybody.domain.Action;
import edu.fudan.fybody.domain.Record;
import edu.fudan.fybody.domain.User;
import edu.fudan.fybody.util.HibernateSessionFactory;

public class RecordAction extends ActionSupport {

	/**
	 * 
	 */
	private static final long serialVersionUID = -1639593596126792543L;
	
	private String uname;
	private String aname;
	private Map<String, String> map;
	
	@JSON(serialize = false)
	public String getUname() {
		return uname;
	}
	public void setUname(String uname) {
		this.uname = uname;
	}
	
	@JSON(serialize = false)
	public String getAname() {
		return aname;
	}
	public void setAname(String aname) {
		this.aname = aname;
	}
	
	public Map<String, String> getResult() {
		return map;
	}
	
	public String execute() throws Exception {
		
		Session session = HibernateSessionFactory.getSession();
		map = new HashMap<String, String>();
		Transaction tx = null;
		
		User user = new User();
		Action action = new Action();
		Record record = new Record();
		
		try {		
			Criteria ucrit = session.createCriteria(User.class);
			ucrit.add(Restrictions.eq("account", uname));
			user = (User) ucrit.uniqueResult();
			
			Criteria acrit = session.createCriteria(Action.class);
			acrit.add(Restrictions.eq("name", aname));
			action = (Action) acrit.uniqueResult();
			
			int uid = user.getId();
			int aid = action.getId();		

			//Ö´ÐÐ²åÈë²Ù×÷
			record.setAction_id(aid);
			record.setUser_id(uid);
			record.setCreated_at(new Date());
			
		    tx = session.beginTransaction();
		    session.save(record);
		    tx.commit();
		    
		    map.put("success", "true");
			
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
