import React from 'react';
import styles from './RosNodeDisplay.module.css';

// Component to display ROS node information
export default function RosNodeDisplay({nodeName, nodeDescription, publishers, subscribers, services}) {
  return (
    <div className={styles.nodeContainer}>
      <div className={styles.nodeName}>
        <strong>{nodeName}</strong>
      </div>
      <div className={styles.nodeDescription}>
        {nodeDescription}
      </div>
      <div className={styles.nodeInterfaces}>
        {publishers && publishers.length > 0 && (
          <div className={styles.interfaceSection}>
            <h4>Publishers:</h4>
            <ul>
              {publishers.map((pub, index) => (
                <li key={index} className={styles.topicItem}>
                  <span className={styles.topicName}>{pub.topic}</span> 
                  <span className={styles.messageType}>[{pub.type}]</span>
                </li>
              ))}
            </ul>
          </div>
        )}
        
        {subscribers && subscribers.length > 0 && (
          <div className={styles.interfaceSection}>
            <h4>Subscribers:</h4>
            <ul>
              {subscribers.map((sub, index) => (
                <li key={index} className={styles.topicItem}>
                  <span className={styles.topicName}>{sub.topic}</span> 
                  <span className={styles.messageType}>[{sub.type}]</span>
                </li>
              ))}
            </ul>
          </div>
        )}
        
        {services && services.length > 0 && (
          <div className={styles.interfaceSection}>
            <h4>Services:</h4>
            <ul>
              {services.map((srv, index) => (
                <li key={index} className={styles.topicItem}>
                  <span className={styles.topicName}>{srv.service}</span> 
                  <span className={styles.messageType}>[{srv.type}]</span>
                </li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </div>
  );
}