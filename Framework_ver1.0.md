[TOC]

## P1

Ok, our group has discussed for several hours. Now we decide to focus on the second challenge in the brief:

- Minimise Carbon Footprint: Integrate route and logistics optimisation algorithms that reduce unnecessary transport distances and associated emissions.



The background is:

- RPM Hire manages a vast fleet of traffic equipment, such as barriers, lighting towers, and electronic signs, for major infrastructure projects. Every day, vehicles are dispatched from a central depot to deliver, collect, and reposition this equipment at various job sites across the city.

Based on this context, we established a question:

- How do we help the target audience (RPM Hire) to optimize the vehicle task assignment, in order to minimize the carbon footprint (i.e. minimize total distance traveled.)

---

Now I describe how we deliver the pitching/presentation:

First, we need to prepare a story. In this story, we describe how the vehicle schedules are made.

- The tasks are scheduled to each vehicle by a manager, who knows the situation, evaluates the task, and decides which tasks are assigned to whom.
- And there exists problems: the schedules may not be optimal to minimize carbon footprint. Furthurmore, when a new task comes up, it is difficult for a human to adjust the current schedule. In most cases, the manager can only do a minor change to current schedule, and hardly make a big, global adjustment that works better.



Based on this, we introduce a system (for now we name it "Automated vehicle planning and management system") that assign the list of tasks to each vehicle and finds the most efficient route.



For this model, we need to make some assumptions and factors (that need to be taken into account) to establish our model, some of these assumptions also brings degree of freedom to our model:

- Road networks;
- Vehicle capacity;
- Time windows for deliveries.
- Each vehicle starts at various job sites across the city;
- There may be multiple depot to embark the loading, so the system needs to decides which vehicle to go to which depot;
- The tasks appears randomly over time, and we may assign new task to vehicle on the road;

Simplifications (ignore the following factors):

- Real-world traffic conditions.



This problem is a classic optimization challenge and there exists several algorithms to do this. Compared to existing solutions, our solutions need to make some improvement and advantages, and we need to address this in the presentation.



----



Then we demonstrate our solution/system. For now we skip this part.



---



Finally, make a summary of the solution.



---





## P2 **Revised Presentation Framework: EcoRoute AI**

**1. The Story: A Day in the Life of a Logistics Manager** 😥

Start with a relatable narrative to create an emotional connection.

- **Introduce a Persona:** "Meet **Sarah**, an operations manager at RPM Hire. Her day is a complex puzzle of last-minute calls, shifting project timelines, and a fleet of vehicles scattered across the city."
- **Describe the Pain Point:** "Currently, Sarah schedules tasks manually. When a new urgent task comes in—like needing a light tower moved across town—she has to quickly find the 'closest' driver. This often means disrupting a carefully planned route for a small change. It's stressful, inefficient, and leads to unnecessary kilometres on the road."
- **State the Core Problem:** "This manual process, repeated daily, results in a **significant and avoidable carbon footprint**. The schedules are rarely optimal, and there's no easy way to make a big, system-wide adjustment that would be truly efficient. This directly challenges RPM Hire's sustainability goals." 

---

**2. The Opportunity: Turning Data into Action** 💡

Connect the problem directly to the specific details in the hackathon brief.

- **The Game Changer:** "RPM Hire is already building the future. By 2026, **90% of its fleet will be GPS-enabled**."  "This creates a massive stream of real-time data, but the question is: how do you use it effectively?"
- **Our Vision:** "We see this not just as data, but as the key to unlocking a new level of operational and environmental intelligence. Our solution turns this raw GPS data into smarter, greener decisions."

------

 **3. Our Solution: Introducing EcoRoute AI** 🚀

Give your system a memorable name and a clear, concise mission statement.

- **The Big Idea:** "We present **EcoRoute AI**, an intelligent routing and logistics platform designed to automate vehicle task assignments for RPM Hire."
- **Mission Statement:** "Our system's core mission is to find the most efficient route for the entire fleet, **minimising total distance travelled to slash the company's carbon footprint**."

------

**4. Key Features & How It Works** ⚙️

Break down what your solution does into tangible features.

- **Dynamic Route Optimization:** EcoRoute AI doesn't just plan the day once. When a new task appears, it instantly recalculates the *entire fleet's schedule* to find the new global optimum, assigning the task to the best-positioned vehicle.
- **Multi-Factor Logistics Engine:** Our model is a sophisticated system that solves a variation of the classic **Vehicle Routing Problem (VRP)**. It considers critical constraints:
  - Vehicle carrying capacity.
  - Multiple depots for loading equipment.
  - Delivery time windows for different job sites.
- **Sustainability Dashboard:** An intuitive interface for Sarah that provides **clear, actionable insights**.  It visualises key metrics like:
  - Kilometres saved per day.
  - Estimated CO₂ emissions reduced.
  - Fleet utilisation rate.

------

**5. The EcoRoute AI Advantage** ✨

This is where you explain why your solution is better than existing ones or a manual approach.

- **Truly Dynamic, Not Static:** Unlike traditional route planners that set a schedule for the day, our system lives and breathes with your operations, adapting in real-time to new information.
- **Sustainability at its Core:** Our primary optimization goal is to reduce emissions, directly supporting **SDGs 9, 11, 12, and 13**.  This isn't just a cost-saving tool; it's an environmental impact tool.
- **Built for the User:** We designed the interface for a non-technical manager like Sarah. It's about providing simple, powerful recommendations, not overwhelming data.

------

**Summary & Impact** 🌍

End with a powerful summary of the benefits for RPM Hire.

- **Greener Operations:** Directly tackles the challenge of minimising the carbon footprint by reducing unnecessary transport. 
- **Smarter Decisions:** Transforms complex fleet data into simple, actionable routing plans that save time and reduce fuel costs.
- **Future-Ready:** Builds a scalable system that leverages RPM Hire's investment in GPS technology, creating a foundation for future innovations like predictive demand planning.



## P3 Slides 1 (Pre-screening)

### **Slide 1: Project Overview**



**Team Name:** [Your Team Name Here]

**Project Title:** EcoRoute AI

------



### **Problem Statement**

How can we help **RPM Hire** to optimize their vehicle route and logistics planning in order to reduce unnecessary transport distances and their associated carbon emissions?

### **Summary of the Solution**

**EcoRoute AI** is an intelligent logistics platform designed to solve this challenge. Our solution leverages the real-time GPS data from RPM Hire's fleet to automate and optimize vehicle routing.



- **Core Function:** The system analyzes all pending tasks and vehicle locations to calculate the most efficient routes for the *entire fleet*, specifically to **minimize total distance travelled**.
- **Dynamic Re-routing:** When a new task is created, our algorithm instantly re-optimizes all routes to seamlessly integrate the new job with maximum efficiency.
- **Impact:** By replacing manual guesswork with data-driven optimization, EcoRoute AI directly reduces carbon emissions, lowers fuel costs, and improves operational agility, providing a clear path to a more sustainable business model.



> [!NOTE]
>
> Sample 2:
>
> EcoRoute AI is an intelligent logistics platform that uses real-time GPS data from RPM Hire’s fleet to automate and optimize vehicle routing. By solving a dynamic Vehicle Routing Problem, our system assigns tasks to the most efficient vehicles in real-time, adapting to new requests as they arise.
>
> The core objective is to minimize the total distance travelled by the entire fleet, which directly reduces fuel consumption and the company's carbon footprint. This provides a clear path for RPM Hire to meet its sustainability goals while also enhancing operational efficiency.