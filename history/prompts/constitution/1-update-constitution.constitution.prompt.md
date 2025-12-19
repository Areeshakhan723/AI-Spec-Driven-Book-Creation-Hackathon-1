---
id: 1
title: Update Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Claude Code
command: /sp.constitution "AI/Spec-Driven Book with Embedded RAG Chatbot"
labels: [constitution,spec-first,rag-chatbot,docusaurus]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
/sp.constitution

Project: AI/Spec-Driven Book with Embedded RAG Chatbot

Core principles:
- Spec-first workflow using Spec-Kit Plus
- Technical accuracy from official sources
- Clear, developer-focused writing
- Reproducible setup and deployment

Key standards:
- Book written with Docusaurus and deployed on GitHub Pages
- RAG chatbot grounded only in book content or user-selected text
- Stack: OpenAI Agents / ChatKit, FastAPI, Neon Postgres, Qdrant Cloud
- Runnable, well-documented code

Constraints:
- GitHub-based source control
- No hallucinated responses
- End-to-end reproducibility

Success criteria:
- Live book on GitHub Pages
- Fully functional embedded RAG chatbot
- All specs implemented via Spec-Kit Plus
```

## Response snapshot

Updated the constitution file with project-specific principles for the AI/Spec-Driven Book with Embedded RAG Chatbot project, including:
- Core principles: Spec-First Workflow, Technical Accuracy, Developer-Focused Writing, Reproducible Setup, GitHub-Based Source Control, No Hallucinated Responses
- Key Standards: Book Platform, RAG Chatbot, Technology Stack, Code Quality
- Constraints: Source Control, Response Integrity, End-to-End Reproducibility
- Success Criteria: Live Deployment, Functional Chatbot, Specification Compliance

## Outcome

- ✅ Impact: Updated constitution with specific project principles and governance
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: Implement features based on updated constitution
- 🧠 Reflection: Constitution now properly defines the project's core values and operational guidelines

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing features according to the new constitution