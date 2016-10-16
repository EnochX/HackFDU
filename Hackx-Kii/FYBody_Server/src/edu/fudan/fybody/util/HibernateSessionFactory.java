package edu.fudan.fybody.util;

import org.hibernate.HibernateException;
import org.hibernate.Session;
import org.hibernate.cfg.Configuration;
import org.hibernate.service.ServiceRegistry;
import org.hibernate.service.ServiceRegistryBuilder;


public class HibernateSessionFactory {
	
  private static String CONFIG_FILE_LOCATION = "/hibernate.cfg.xml";
  private static final ThreadLocal<Session>threadLocal = new ThreadLocal<Session>();
  private static Configuration configuration = new Configuration();
  private static org.hibernate.SessionFactory sessionFactory;
  private static String configFile = CONFIG_FILE_LOCATION;
  private static ServiceRegistry serviceRegistry;
  
  static{
	  try{
		  configuration.configure(configFile);
		  serviceRegistry = new ServiceRegistryBuilder().applySettings(configuration.getProperties()).buildServiceRegistry();
		  sessionFactory = configuration.buildSessionFactory(serviceRegistry);
	  } catch(Exception e) {
		  System.err.println("%%% Error Creating SessionFactory %%%");
		  e.printStackTrace();
	  }
  }
  
  private HibernateSessionFactory(){
	  
  }
  
  public static Session getSession() throws HibernateException{
	  Session session = (Session)threadLocal.get();
	  if(session == null || !session.isOpen()){
		  if(sessionFactory == null){
			  rebuildSessionFactory();
		  }
		  session = (sessionFactory != null) ? sessionFactory.openSession() : null;
		  threadLocal.set(session);
	  }
	  return session;
  }
  
  public static void rebuildSessionFactory(){
	  try{
		  configuration.configure(configFile);
		  serviceRegistry = new ServiceRegistryBuilder().applySettings(configuration.getProperties()).buildServiceRegistry();
		  sessionFactory = configuration.buildSessionFactory(serviceRegistry);
	  } catch(Exception e) {
		  System.err.println("%%%%Error Creating SessionFactory%%%%");
		  e.printStackTrace();
	  }
  }
  
  public static void closeSession() throws HibernateException{
	  Session session = (Session) threadLocal.get();
	  threadLocal.set(null);
	  if(session != null){
		  session.close();
	  }
  }
  
  public static org.hibernate.SessionFactory getSessionFactory(){
	  return sessionFactory;
  }
  
  public static void setConfigFile(String configFile){
	  HibernateSessionFactory.configFile = configFile;
	  sessionFactory = null;
  }
  
  public static Configuration getConfiguration(){
	  return configuration;
  }
}
