import React, { useState } from 'react';
import styles from './AssessmentForm.module.css';

// Component for course assessment forms
export default function AssessmentForm({title, questions, onSubmit}) {
  const [answers, setAnswers] = useState({});
  const [submitted, setSubmitted] = useState(false);
  
  const handleAnswerChange = (questionId, value) => {
    setAnswers(prev => ({
      ...prev,
      [questionId]: value
    }));
  };
  
  const handleSubmit = (event) => {
    event.preventDefault();
    if (onSubmit) {
      onSubmit(answers);
    }
    setSubmitted(true);
  };
  
  if (submitted) {
    return (
      <div className={styles.assessmentContainer}>
        <h3>{title}</h3>
        <div className={styles.submissionMessage}>
          Thank you for submitting the assessment!
        </div>
      </div>
    );
  }
  
  return (
    <div className={styles.assessmentContainer}>
      <h3>{title}</h3>
      <form onSubmit={handleSubmit} className={styles.assessmentForm}>
        {questions.map((question, index) => (
          <div key={question.id || index} className={styles.question}>
            <label className={styles.questionLabel}>
              {question.text}
              {question.required && <span className={styles.required}> *</span>}
            </label>
            
            {question.type === 'text' && (
              <textarea
                className={styles.textarea}
                onChange={(e) => handleAnswerChange(question.id, e.target.value)}
                required={question.required}
                rows={question.rows || 4}
              />
            )}
            
            {question.type === 'multiple-choice' && (
              <div className={styles.choices}>
                {question.options?.map((option, optIndex) => (
                  <label key={optIndex} className={styles.choiceLabel}>
                    <input
                      type="radio"
                      name={question.id}
                      value={option.value}
                      onChange={(e) => handleAnswerChange(question.id, e.target.value)}
                      required={question.required}
                    />
                    {option.label}
                  </label>
                ))}
              </div>
            )}
            
            {question.type === 'scale' && (
              <div className={styles.scale}>
                {[1, 2, 3, 4, 5].map(value => (
                  <label key={value} className={styles.scaleLabel}>
                    <input
                      type="radio"
                      name={question.id}
                      value={value}
                      onChange={(e) => handleAnswerChange(question.id, e.target.value)}
                      required={question.required}
                    />
                    {value}
                  </label>
                ))}
                <div className={styles.scaleLabels}>
                  <span>Poor</span>
                  <span>Excellent</span>
                </div>
              </div>
            )}
          </div>
        ))}
        
        <button type="submit" className={styles.submitButton}>
          Submit Assessment
        </button>
      </form>
    </div>
  );
}