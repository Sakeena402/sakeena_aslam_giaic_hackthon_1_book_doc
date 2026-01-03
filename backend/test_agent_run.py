#python backend/test_agent_run.py

from agent import RAGAgent

def main():
    agent = RAGAgent.from_config()

    response = agent.process_query(
        query_text="Explain VLA models",
        query_type="global",
        top_k=3
    )

    print("\n--- AGENT ANSWER ---\n")
    print(response.response_text)
    print("\nConfidence:", response.confidence_score)
    print("Chunks used:", len(response.grounding_chunks))

if __name__ == "__main__":
    main()
