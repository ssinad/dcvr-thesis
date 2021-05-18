FROM registry.gitlab.com/ssinad/column-generation/cplex

COPY .github/scripts/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]