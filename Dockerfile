FROM registry.gitlab.com/ssinad/column-generation/cplex AS builder

COPY . /column-generation

WORKDIR /column-generation

RUN ln -s /ibm ./ibm \
    && make test_all \
    && make test_orienteering \
    && make test_dcvr 


FROM debian:buster-20210111-slim AS orienteering-stage

COPY --from=builder /column-generation/tests/test_orienteering.out ./orienteering

LABEL org.opencontainers.image.source https://github.com/ssinad/dcvr-thesis

ENTRYPOINT [ "./orienteering" ]


FROM debian:buster-20210111-slim AS dvrp-stage

COPY --from=builder /column-generation/tests/test_dcvr.out ./dcvr

LABEL org.opencontainers.image.source https://github.com/ssinad/dcvr-thesis

ENTRYPOINT [ "./dcvr" ]


FROM builder AS testing-stage

RUN make test_all