---
applyTo: '**'
---

### **Please Ensure:**

#### **1. Thinking & Reasoning**

- **Use Chain-of-Thought (CoT):** Before providing the final code, outline a **step-by-step plan** detailing the logic and approach.
- **Apply Self-Consistency:** Generate multiple approaches and select the most effective one based on performance, readability, and maintainability.
- **Leverage Debate Prompting:** Compare at least two possible implementations, discussing their trade-offs (e.g., performance vs. readability).
- **No Quick and Easy Approach:** Do not create any simplified approach for the sake of finishing the task. It is absolutely important to provided high standard approach (rather than quick and dirty). This does not mean over coding. It only means implementing a feature correctly which might require patience.

#### **2. State-of-the-Art Solution**

- Utilize **the latest best practices and advanced techniques** to ensure optimal performance and maintainability.
- **Apply Least-to-Most Prompting:** Break the problem into simpler subproblems before solving the full problem.
- **Use Auto-CoT:** Develop a **structured plan**, review it, and refine the solution before finalizing the code.

#### **3. Code Efficiency**

- Write **concise, straightforward** code that minimizes redundancy.
- Ensure the logic is implemented **with the fewest lines possible** while maintaining clarity.
- **KISS (Keep It Simple, Stupid)**: Prioritize the simplest solution that fully meets requirements; avoid over-engineering.
- **SRP (Single Responsibility Principle)**: Ensure each class, function, or module has one clear responsibility.
- **DRY (Don't Repeat Yourself)**: Eliminate duplication in code, configurations, or data.
- **YAGNI (You Aren't Gonna Need It)**: Only implement features that are explicitly required.

#### **4. Documentation**

- Include **clear and concise comments** and **docstrings** to explain the purpose and functionality of each function/class.
- Use for all classes, functions, and public APIs (include params, returns, raises/exceptions, examples).
- **Project Documentation**: Maintain a `/docs` folder with **Setup instructions** (e.g., installation, environment setup), **Usage examples** (e.g., code snippets, CLI commands), **API references** (auto-generated where possible).
- **Architecture Diagrams**: Include high-level diagrams (e.g., using Mermaid or PlantUML) for system overview.

#### **5. Error Handling & Edge Cases**

- Implement **error handling** to gracefully manage potential exceptions.
- Consider **edge cases and boundary conditions** to prevent unexpected failures.
- Use **logger** for better error handling.

#### **6. Performance Optimization**

- Optimize for **speed and resource efficiency** while ensuring maintainability.
- Avoid unnecessary loops, redundant calculations, or excessive memory usage.

#### **7. Best Practices & Scalability**

- Follow **coding standards and best practices** for the given programming language.
- Ensure the solution is **modular and reusable** to support future extensions.
  
#### **8. Testing**

- Write unit testing for every function and class.
- Use the exact same name (if possible) as the function and classes with added "test-" to the begining. This helps navigating.
  
#### **9. Project Management**

- Keep track of issues you encounter inside `issues.md` and keep it updated.
- **Changelog**: Maintain a `CHANGELOG.md` for versioned changes.